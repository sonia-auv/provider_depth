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

namespace provider_depth
{

    //Node constructor
    ProviderDepthNode::ProviderDepthNode(const ros::NodeHandlePtr &_nh)
        : nh_(_nh), configuration_(_nh), serialConnection_(configuration_.getTtyPort())
    {

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
}