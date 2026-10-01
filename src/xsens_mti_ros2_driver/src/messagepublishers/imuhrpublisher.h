//  Copyright (c) 2003-2024 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions, and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions, and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the names of the copyright holders nor the names of their contributors
//     may be used to endorse or promote products derived from this software without
//     specific prior written permission.

#ifndef IMUHRPUBLISHER_H
#define IMUHRPUBLISHER_H

#include "packetcallback.h"
#include "publisherhelperfunctions.h"

#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>

struct ImuHRPublisher : public PacketCallback, PublisherHelperFunctions
{
    struct TimedVector
    {
        uint32_t sample_time_fine;
        std::array<double, 3> value;
        rclcpp::Time stamp;
    };

    DriverPublisher<sensor_msgs::msg::Imu> pub;
    DriverNode::SharedPtr node;
    double linear_acceleration_variance[3];
    double angular_velocity_variance[3];
    std::string frame_id = DEFAULT_FRAME_ID;

    std::deque<TimedVector> acceleration_buffer;
    std::deque<TimedVector> gyro_buffer;

    static constexpr std::size_t MAX_BUFFER_SIZE = 64;
    static constexpr int SAMPLE_TIME_FINE_HZ = 10000;

    int pair_tolerance_ticks = 20;

    explicit ImuHRPublisher(DriverNode::SharedPtr node_handle)
        : node(node_handle)
    {
        std::vector<double> variance = {0.0, 0.0, 0.0};

        if (!node->has_parameter("angular_velocity_stddev"))
            node->declare_parameter("angular_velocity_stddev", variance);
        if (!node->has_parameter("linear_acceleration_stddev"))
            node->declare_parameter("linear_acceleration_stddev", variance);

        int pub_queue_size = 5;
        node->get_parameter("publisher_queue_size", pub_queue_size);
        pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu/data_hr", pub_queue_size);

        node->get_parameter("frame_id", frame_id);

        variance_from_stddev_param(
            "angular_velocity_stddev", angular_velocity_variance, node);
        variance_from_stddev_param(
            "linear_acceleration_stddev", linear_acceleration_variance, node);

        int accel_rate = 500;
        int gyro_rate = 500;
        node->get_parameter("output_data_rate_acchr", accel_rate);
        node->get_parameter("output_date_rate_gyrohr", gyro_rate);

        const int reference_rate = std::max(1, std::min(accel_rate, gyro_rate));
        pair_tolerance_ticks = std::max(
            1,
            static_cast<int>(std::ceil(
                static_cast<double>(SAMPLE_TIME_FINE_HZ) /
                static_cast<double>(reference_rate))));

        RCLCPP_INFO(
            node->get_logger(),
            "/imu/data_hr enabled: AccHR=%d Hz, GyroHR=%d Hz, pairing tolerance=%d SampleTimeFine ticks",
            accel_rate, gyro_rate, pair_tolerance_ticks);
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        if (!packet.containsSampleTimeFine())
        {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 5000,
                "/imu/data_hr requires SampleTimeFine in the Xsens output configuration.");
            return;
        }

        const uint32_t sample_time_fine = packet.sampleTimeFine();

        if (packet.containsAccelerationHR())
        {
            const XsVector accel_hr = packet.accelerationHR();
            acceleration_buffer.push_back({
                sample_time_fine,
                {accel_hr[0], accel_hr[1], accel_hr[2]},
                timestamp});
        }

        if (packet.containsRateOfTurnHR())
        {
            const XsVector gyro_hr = packet.rateOfTurnHR();
            gyro_buffer.push_back({
                sample_time_fine,
                {gyro_hr[0], gyro_hr[1], gyro_hr[2]},
                timestamp});
        }

        publishAvailablePairs();
        trimBuffers();
    }

private:
    static int64_t signedTickDifference(uint32_t from, uint32_t to)
    {
        return static_cast<int64_t>(static_cast<int32_t>(to - from));
    }

    void publishAvailablePairs()
    {
        while (!acceleration_buffer.empty() && !gyro_buffer.empty())
        {
            const TimedVector &accel = acceleration_buffer.front();

            auto best_gyro = gyro_buffer.end();
            int64_t best_abs_diff = std::numeric_limits<int64_t>::max();

            for (auto it = gyro_buffer.begin(); it != gyro_buffer.end(); ++it)
            {
                const int64_t diff = signedTickDifference(
                    accel.sample_time_fine, it->sample_time_fine);
                const int64_t abs_diff = std::llabs(diff);

                if (abs_diff < best_abs_diff)
                {
                    best_abs_diff = abs_diff;
                    best_gyro = it;
                }
            }

            if (best_gyro != gyro_buffer.end() &&
                best_abs_diff <= pair_tolerance_ticks)
            {
                publishImu(accel, *best_gyro, best_abs_diff);
                acceleration_buffer.pop_front();
                gyro_buffer.erase(best_gyro);
                continue;
            }

            // If the oldest acceleration sample is already more than one
            // nominal sample period older than the oldest gyro sample, it can
            // no longer form the closest pair. Drop it and continue.
            const int64_t oldest_diff = signedTickDifference(
                accel.sample_time_fine, gyro_buffer.front().sample_time_fine);

            if (oldest_diff > pair_tolerance_ticks)
            {
                RCLCPP_WARN_THROTTLE(
                    node->get_logger(), *node->get_clock(), 5000,
                    "Dropping unmatched AccelerationHR sample. Acc=%u, oldest Gyro=%u, delta=%ld ticks",
                    accel.sample_time_fine,
                    gyro_buffer.front().sample_time_fine,
                    static_cast<long>(oldest_diff));
                acceleration_buffer.pop_front();
                continue;
            }

            // Gyro samples that are too old for the current acceleration sample
            // cannot match a later acceleration sample either.
            if (oldest_diff < -pair_tolerance_ticks)
            {
                gyro_buffer.pop_front();
                continue;
            }

            // Need one more packet before deciding.
            break;
        }
    }

    void publishImu(
        const TimedVector &accel,
        const TimedVector &gyro,
        int64_t pair_delta_ticks)
    {
        sensor_msgs::msg::Imu msg;

        // AccelerationHR defines the output timeline. FAST-LIO consumes
        // angular velocity and linear acceleration from this message.
        msg.header.stamp = accel.stamp;
        msg.header.frame_id = frame_id;

        msg.linear_acceleration.x = accel.value[0];
        msg.linear_acceleration.y = accel.value[1];
        msg.linear_acceleration.z = accel.value[2];

        msg.angular_velocity.x = gyro.value[0];
        msg.angular_velocity.y = gyro.value[1];
        msg.angular_velocity.z = gyro.value[2];

        // No high-rate orientation is supplied by this publisher.
        msg.orientation_covariance[0] = -1.0;

        msg.angular_velocity_covariance[0] = angular_velocity_variance[0];
        msg.angular_velocity_covariance[4] = angular_velocity_variance[1];
        msg.angular_velocity_covariance[8] = angular_velocity_variance[2];

        msg.linear_acceleration_covariance[0] = linear_acceleration_variance[0];
        msg.linear_acceleration_covariance[4] = linear_acceleration_variance[1];
        msg.linear_acceleration_covariance[8] = linear_acceleration_variance[2];

        pub->publish(msg);

        RCLCPP_DEBUG_THROTTLE(
            node->get_logger(), *node->get_clock(), 2000,
            "/imu/data_hr publishing; Acc=%u, Gyro=%u, delta=%ld ticks",
            accel.sample_time_fine,
            gyro.sample_time_fine,
            static_cast<long>(pair_delta_ticks));
    }

    void trimBuffers()
    {
        while (acceleration_buffer.size() > MAX_BUFFER_SIZE)
            acceleration_buffer.pop_front();
        while (gyro_buffer.size() > MAX_BUFFER_SIZE)
            gyro_buffer.pop_front();
    }
};

#endif
