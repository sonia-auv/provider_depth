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

        ros::NodeHandlePtr nh_;
        Configuration configuration_;
        Serial serialConnection_;

        std::string id1_string;

        std::thread readThread;
        std::mutex id1_mutex;
        std::condition_variable id1_cond;
};

}

#endif //PROVIDER_DEPTH_NODE