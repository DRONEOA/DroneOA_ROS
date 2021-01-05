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

#ifndef DATAPOOL_SUBSCRIBER_  // NOLINT
#define DATAPOOL_SUBSCRIBER_  // NOLINT

#include <string>
#include <boost/any.hpp>
#include <droneoa_ros/Utils/DataPoolEntries.hpp>

namespace DP {

class DataPoolSubscriber {
    ENTRY_TYPES mTypeToSubscribe;
 public:
    DataPoolSubscriber();
    explicit DataPoolSubscriber(ENTRY_TYPES type);
    virtual ~DataPoolSubscriber();
    /**
     * @brief Set the Type of DP Changes To Subscribe
     * @param type choose from DATA / CONFIG / ALL / NONE
     */
    void setTypeToSubscribe(ENTRY_TYPES type);
    /**
     * @brief Get the Type of DP Changes To Subscribe
     * @return ENTRY_TYPES current subscribed DP change type
     */
    ENTRY_TYPES getTypeToSubscribe();
    /**
     * @brief Event OnDataPoolUpdate; Default behavior is do nothing
     * @param entryName updated entry name
     * @param data updated data
     */
    virtual void onDataPoolUpdate(std::string entryName, boost::any data);
};

}  // namespace DP

#endif  // DATAPOOL_SUBSCRIBER_  // NOLINT
