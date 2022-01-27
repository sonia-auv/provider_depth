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

#define BUFFER_SIZE 4096

namespace provider_depth
{

    //Node constructor
    ProviderDepthNode::ProviderDepthNode(const ros::NodeHandlePtr &_nh)
        : nh_(_nh), configuration_(_nh), serialConnection_(configuration_.getTtyPort())
    {
        depthPublisher_ = nh_->advertise<std_msgs::Float32>("/provider_depth/depth", 100);
        
        readThread = std::thread(std::bind(&ProviderDepthNode::readSerialDevice, this));
        sendID1Thread = std::thread(std::bind(&ProviderDepthNode::sendId1Register, this));

        serialConnection_.flush();
    }

    ProviderDepthNode::~ProviderDepthNode()
    {
        readThread.~thread();
    }

    void ProviderDepthNode::Spin()
    {
        ros::Rate r(100); // 100 Hz

        while(ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    void ProviderDepthNode::readSerialDevice()
    {
        ros::Rate r(100); // 100 Hz
        char buffer[BUFFER_SIZE];
        ROS_INFO_STREAM("Serial Read Thread is started");

        while(!ros::isShuttingDown())
        {
            // find the message beginning
            do
            {
                serialConnection_.readOnce(buffer, 0);
            }while(buffer[0] != '$');

            int i;

            for(i = 1; buffer[i-1] != '\n' && i < BUFFER_SIZE; i++)
            {
                serialConnection_.readOnce(buffer, i);
            }

            if(i >= BUFFER_SIZE)
            {
                continue;
            }

            buffer[i] = 0;

            if(!strncmp(&buffer[1], ID1, 5)) // Add checksum verification
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

        while(!ros::isShuttingDown())
        {
            tmp = "";
            std::unique_lock<std::mutex> mlock(id1_mutex);
            id1_cond.wait(mlock);

            try
            {
                if(!id1_string.empty()) // Add checksum verification
                {
                    std::stringstream ss(id1_string);

                    std::getline(ss, tmp, ','); // Get the header of the message

                    std::getline(ss, tmp, ','); // Get the depth
                    depth_.data = stof(tmp);
                    depthPublisher_.publish(depth_);


                    std::getline(ss, tmp, ','); // M

                    std::getline(ss, tmp, ','); // Get the pressure

                    depth_.data = stof(tmp);

                    depthPublisher_.publish(depth_);



                    std::getline(ss, tmp, ','); // B

                    std::getline(ss, tmp, ','); // Get the temperature

                    depth_.data = stof(tmp);

                    depthPublisher_.publish(depth_);
                }
            }
            catch(...)
            {
                ROS_ERROR_STREAM("Issue with the message received from depth sensor");
            }
            
        }
    }
}