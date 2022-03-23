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

#ifndef PROVIDER_DEPTH_NODE
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

#define ID1 "ISDPT"

namespace provider_depth {

class ProviderDepthNode
{
    public:

        ProviderDepthNode(const ros::NodeHandlePtr &_nh);
        ~ProviderDepthNode();

        void Spin();

    private:

        void readSerialDevice();
        void sendId1Register();

        ros::NodeHandlePtr nh_;
        Configuration configuration_;
        Serial serialConnection_;

        ros::Publisher depthPublisher_;
        ros::Publisher pressPublisher_;
        ros::Publisher tempPublisher_;

        std::thread readThread;
        std::thread sendID1Thread;
        std::mutex id1_mutex;
        std::string id1_string = "";
        std::condition_variable id1_cond;

        std_msgs::Float32 depth_;
        std_msgs::Float32 press_;
        std_msgs::Float32 temp_;

        bool tare(std_srvs::Empty::Request &tareRsq, std_srvs::Empty::Response &tareRsp);
        ros::ServiceServer tare_srv;
};

}

#endif //PROVIDER_DEPTH_NODE