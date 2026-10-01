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

struct ImuHRPublisher : public PacketCallback, PublisherHelperFunctions
{
    DriverPublisher<sensor_msgs::msg::Imu> pub;
    double linear_acceleration_variance[3];
    double angular_velocity_variance[3];
    std::string frame_id = DEFAULT_FRAME_ID;

    explicit ImuHRPublisher(DriverNode::SharedPtr node)
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
        // Publish only when acceleration and rate-of-turn belong to the same
        // Xsens data packet. This avoids combining measurements from different
        // sample times.
        if (!packet.containsAccelerationHR() || !packet.containsRateOfTurnHR())
            return;

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = timestamp;
        msg.header.frame_id = frame_id;

        const XsVector accel_hr = packet.accelerationHR();
        const XsVector gyro_hr = packet.rateOfTurnHR();

        msg.linear_acceleration.x = accel_hr[0];
        msg.linear_acceleration.y = accel_hr[1];
        msg.linear_acceleration.z = accel_hr[2];

        msg.angular_velocity.x = gyro_hr[0];
        msg.angular_velocity.y = gyro_hr[1];
        msg.angular_velocity.z = gyro_hr[2];

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
    }
};

#endif
