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
        readThread = std::thread(std::bind(&ProviderDepthNode::readSerialDevice, this));
    }

    ProviderDepthNode::~ProviderDepthNode()
    {

    }

    void ProviderDepthNode::Spin()
    {
        ros::Rate r(50); // 50 Hz

        while(ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }
    }

    void ProviderDepthNode::readSerialDevice()
    {
        ros::Rate r(100);
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
        }
    }
}