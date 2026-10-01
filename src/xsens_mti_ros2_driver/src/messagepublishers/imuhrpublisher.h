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
#include <cstdint>
#include <deque>

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
    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp) override
    {
        // AccelerationHR and RateOfTurnHR may arrive in separate Xsens packets.
        // SampleTimeFine is used to pair measurements that belong to the same
        // sensor sample before publishing one sensor_msgs/Imu message.
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
            storeSample(
                acceleration_buffer,
                sample_time_fine,
                {accel_hr[0], accel_hr[1], accel_hr[2]},
                timestamp);
        }

        if (packet.containsRateOfTurnHR())
        {
            const XsVector gyro_hr = packet.rateOfTurnHR();
            storeSample(
                gyro_buffer,
                sample_time_fine,
                {gyro_hr[0], gyro_hr[1], gyro_hr[2]},
                timestamp);
        }

        publishIfMatched(sample_time_fine);
        trimBuffers();
    }

private:
    static void storeSample(
        std::deque<TimedVector> &buffer,
        uint32_t sample_time_fine,
        const std::array<double, 3> &value,
        rclcpp::Time stamp)
    {
        auto existing = std::find_if(
            buffer.begin(), buffer.end(),
            [sample_time_fine](const TimedVector &sample)
            {
                return sample.sample_time_fine == sample_time_fine;
            });

        if (existing != buffer.end())
        {
            existing->value = value;
            existing->stamp = stamp;
            return;
        }

        buffer.push_back({sample_time_fine, value, stamp});
    }

    void publishIfMatched(uint32_t sample_time_fine)
    {
        auto accel_it = std::find_if(
            acceleration_buffer.begin(), acceleration_buffer.end(),
            [sample_time_fine](const TimedVector &sample)
            {
                return sample.sample_time_fine == sample_time_fine;
            });

        auto gyro_it = std::find_if(
            gyro_buffer.begin(), gyro_buffer.end(),
            [sample_time_fine](const TimedVector &sample)
            {
                return sample.sample_time_fine == sample_time_fine;
            });

        if (accel_it == acceleration_buffer.end() || gyro_it == gyro_buffer.end())
            return;

        const TimedVector accel = *accel_it;
        const TimedVector gyro = *gyro_it;

        sensor_msgs::msg::Imu msg;
        // AccelerationHR is used as the reference timeline.
        msg.header.stamp = accel.stamp;
        msg.header.frame_id = frame_id;

        msg.linear_acceleration.x = accel.value[0];
        msg.linear_acceleration.y = accel.value[1];
        msg.linear_acceleration.z = accel.value[2];

        msg.angular_velocity.x = gyro.value[0];
        msg.angular_velocity.y = gyro.value[1];
        msg.angular_velocity.z = gyro.value[2];

        // High-rate Xsens output contains acceleration and rate of turn only.
        // Mark orientation as unavailable according to sensor_msgs/Imu.
        msg.orientation_covariance[0] = -1.0;

        msg.angular_velocity_covariance[0] = angular_velocity_variance[0];
        msg.angular_velocity_covariance[4] = angular_velocity_variance[1];
        msg.angular_velocity_covariance[8] = angular_velocity_variance[2];

        msg.linear_acceleration_covariance[0] = linear_acceleration_variance[0];
        msg.linear_acceleration_covariance[4] = linear_acceleration_variance[1];
        msg.linear_acceleration_covariance[8] = linear_acceleration_variance[2];

        pub->publish(msg);

        acceleration_buffer.erase(accel_it);
        gyro_buffer.erase(gyro_it);
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
