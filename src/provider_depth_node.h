/**
 * \file	provider_depth_node.h
 * \author	Francis Alonzo
 * \date	24/07/2017
 *
 * \copyright Copyright (c) 2021 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROVIDER_DEPTH_NODE // replace with pragma once (optional)
#define PROVIDER_DEPTH_NODE

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <std_msgs/Float32.h>

#include <std_srvs/Empty.h>

#include "Configuration.h"
#include "drivers/serial.h"

#define ID1 "ISDPT" // FIXME: Make const

namespace provider_depth
{

    class ProviderDepthNode
    {
    public:
        /// @brief Create Node
        /// @param _nh
        ProviderDepthNode(const ros::NodeHandlePtr &_nh);
        ~ProviderDepthNode();

        /// @brief Method to start node.
        void Spin();

    private:
        /// @brief Method to read from serial device
        void readSerialDevice();

        /// @brief Check sum thingy.
        void sendId1Register();

        /// @brief Tare the Depth Sensor.
        /// @param tareRsq The request to tare.
        /// @param tareRsp The response from the tare.
        /// @return True of Tare if successful else false.
        bool tare(std_srvs::Empty::Request &tareRsq, std_srvs::Empty::Response &tareRsp);

        /// @brief Node handler for ROS
        ros::NodeHandlePtr nh_;       // FIXME: Start with m not end with _
        
        /// @brief Configuration file. TODO: See if this can be removed.
        Configuration configuration_; // FIXME: Start with m not end with _
        
        /// @brief Serial Connection. TODO: See if this can be part of the constructor.
        Serial serialConnection_;     // FIXME: Start with m not end with _

        /* TODO: Add a description for each publisher */
        ros::Publisher depthPublisher_; // FIXME: Start with m not end with _
        ros::Publisher pressPublisher_; // FIXME: Start with m not end with _
        ros::Publisher tempPublisher_;  // FIXME: Start with m not end with _

        /// @brief Thread for serial connection.
        std::thread readThread;           // FIXME: Start with m
        
        /// @brief TODO: Not sure what this is for
        std::thread sendID1Thread;        // FIXME: Start with m
        std::mutex id1_mutex;             // FIXME: Start with m, make camel case
        std::string id1_string = "";      // FIXME: Start with m, make camel case
        std::condition_variable id1_cond; // FIXME: Start with m, make camel case

        /* Messages */
        std_msgs::Float32 depth_; // FIXME: Start with m not end with _
        std_msgs::Float32 press_; // FIXME: Start with m not end with _
        std_msgs::Float32 temp_;  // FIXME: Start with m not end with _

        /// @brief Service that initiates tare method.
        ros::ServiceServer tare_srv; // FIXME: Start with m, make camel case
    };

}

#endif // PROVIDER_DEPTH_NODE