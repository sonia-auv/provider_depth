/**
 * \file	provider_depth_node.cc
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

#include "provider_depth_node.h"

#define BUFFER_SIZE 4096 // FIXME: make constant

namespace provider_depth
{
    ProviderDepthNode::ProviderDepthNode(const ros::NodeHandlePtr &_nh)
        : nh_(_nh), configuration_(_nh), serialConnection_(configuration_.getTtyPort())
    {
        depthPublisher_ = nh_->advertise<std_msgs::Float32>("/provider_depth/depth", 100);
        pressPublisher_ = nh_->advertise<std_msgs::Float32>("/provider_depth/press", 100);
        tempPublisher_ = nh_->advertise<std_msgs::Float32>("/provider_depth/temp", 100);

        // Thread for Seral Device connection.
        readThread = std::thread(std::bind(&ProviderDepthNode::readSerialDevice, this));
        // Thread for CheckSum function thingy.
        sendID1Thread = std::thread(std::bind(&ProviderDepthNode::sendId1Register, this));

        serialConnection_.flush();

        // Start host server.
        tare_srv = nh_->advertiseService("/provider_depth/tare", &ProviderDepthNode::tare, this);
    }

    ProviderDepthNode::~ProviderDepthNode()
    {
        readThread.~thread();
    }

    void ProviderDepthNode::Spin()
    {
        // TODO: Make the 100 some kind of const or part of the ctor.
        // r is a bad variable name.
        ros::Rate r(100); // 100 Hz

        while (ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    void ProviderDepthNode::readSerialDevice()
    {
        // TODO: Make the 100 some kind of const or part of the ctor.
        // r is a bad variable name.
        ros::Rate r(100); // 100 Hz
        char buffer[BUFFER_SIZE];
        ROS_INFO_STREAM("Serial Read Thread is started");

        while (!ros::isShuttingDown())
        {
            // find the message beginning
            do
            {
                serialConnection_.readOnce(buffer, 0);
            } while (buffer[0] != '$');

            // internal counter
            int i;

            /**
             * Set the counter to 1,
             * While the last index of the buffer is not '\n' AND the counter is less than the buffer
             * post-increment i
             *
             * This fills the buffer one char at a time and checks against
             * the end char or max buffer size.
             */
            for (i = 1; buffer[i - 1] != '\n' && i < BUFFER_SIZE; i++)
            {
                serialConnection_.readOnce(buffer, i);
            }

            // if the buffer is maxed out, skip the rest of the while loop.
            if (i >= BUFFER_SIZE)
            {
                continue;
            }

            // set the last char (ideally the '\n') to 0
            buffer[i] = 0;

            if (!strncmp(&buffer[1], ID1, 5)) // Add checksum verification
            {
                std::unique_lock<std::mutex> mlock(id1_mutex);
                id1_string = std::string(buffer);
                id1_cond.notify_one();
            }

            r.sleep();
        }
    }

    void ProviderDepthNode::sendId1Register()
    {
        std::string tmp = "";

        ROS_INFO_STREAM("Send ID1 Thread started");

        // Why !ros::isShuttingDown() and not ros::ok()
        while (!ros::isShuttingDown())
        {
            tmp = "";
            std::unique_lock<std::mutex> mlock(id1_mutex);
            id1_cond.wait(mlock);

            try
            {
                if (!id1_string.empty()) // Add checksum verification
                {
                    std::stringstream ss(id1_string);

                    std::getline(ss, tmp, ','); // Get the header of the message

                    std::getline(ss, tmp, ','); // Get the depth
                    depth_.data = stof(tmp);
                    depthPublisher_.publish(depth_);

                    std::getline(ss, tmp, ','); // skip M

                    std::getline(ss, tmp, ','); // Get the pressure
                    press_.data = stof(tmp);
                    pressPublisher_.publish(press_);

                    std::getline(ss, tmp, ','); // skip B

                    std::getline(ss, tmp, ','); // Get the temperature
                    temp_.data = stof(tmp);
                    tempPublisher_.publish(temp_);
                }
            }
            catch (...) // BAD Add Exception
            {
                ROS_ERROR_STREAM("Issue with the message received from depth sensor");
            }
        }
    }

    bool ProviderDepthNode::tare(std_srvs::Empty::Request &tareRsq, std_srvs::Empty::Response &tareRsp)
    {
        serialConnection_.transmit("#tare\n");
        ros::Duration(0.1).sleep();

        // Confirmation of tare?

        ROS_INFO("Depth Sensor tare finished");
        return true;
    }
}
