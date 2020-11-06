/* Copyright (C) 2020 DroneOA Group - All Rights Reserved
 * This file is part of DroneOA_ROS.
 *
 * DroneOA_ROS is free software: you can redistribute it and/or 
 * modify it under the terms of the GNU Affero General Public License
 * as published by the Free Software Foundation.
 *
 * DroneOA_ROS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with DroneOA_ROS. 
 * If not, see <https://www.gnu.org/licenses/>.
 *
 * Written by Bohan Shi <b34shi@uwaterloo.ca>, Oct 2020
 */

#include <droneoa_ros/Utils/DataPoolSubscriber.hpp>
#include <droneoa_ros/Utils/DataPool.hpp>

namespace DP {

DataPoolSubscriber::DataPoolSubscriber() : mTypeToSubscribe(ENTRY_TYPES::NONE) {}

DataPoolSubscriber::DataPoolSubscriber(ENTRY_TYPES type) : mTypeToSubscribe(type) {}

DataPoolSubscriber::~DataPoolSubscriber() {}

void DataPoolSubscriber::setTypeToSubscribe(ENTRY_TYPES type) {
    mTypeToSubscribe = type;
}

ENTRY_TYPES DataPoolSubscriber::getTypeToSubscribe() {
    return mTypeToSubscribe;
}

void DataPoolSubscriber::onDataPoolUpdate(std::string entryName) {}

}  // namespace DP
