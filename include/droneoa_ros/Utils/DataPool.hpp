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

#ifndef UTILS_DATAPOOL_  // NOLINT
#define UTILS_DATAPOOL_  // NOLINT

#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>
#include <mutex>  // NOLINT
#include <boost/any.hpp>
#include <droneoa_ros/Utils/DataPoolEntries.hpp>

namespace DP {

/*******************************************************************************
 * Subscriber Class
 */
class DataPoolSubscriber;

/*******************************************************************************
 * Helpers
 */
// Check if a any can be cast to certain type，RTTI
template<typename T>
bool can_cast_to(const boost::any& a) {
    return a.type() == typeid(T);
}

/*******************************************************************************
 * Data Pool
 *! @note Data may not be available during construction. Note construct order.
 *! @note This should be used mainly for:
 *          1. Configurations
 *          2. Low update frequency data that require cross-module access
 *          3. Important flags for debugging
 *          4. Important flags that developers may need
 */
class DataPool {
 protected:
    static std::mutex mContainerMutex;
    static std::map<std::string, boost::any> mDataPoolContainer;
    static std::vector<DataPoolSubscriber*> mSubscriberList;
    void notifyAll(ENTRY_TYPES type, std::string entryName);

 public:
    /**
     * @brief Get the Data object at desired entry
     * @param name name of the data entry
     * @return boost::any data (Note: you can only cast data to it's original type)
     */
    virtual boost::any getData(std::string name);
    /**
     * @brief Get the Data As String object, auto conversion
     * @param name name of the data entry
     * @return std::string 
     */
    virtual std::string getDataAsString(std::string name);
    /**
     * @brief Get the Data As INT
     * @param name name of the data entry
     * @return int
     */
    virtual int getDataAsInt(std::string name);
    /**
     * @brief Set the Data object at desired entry. Subscribers of DATA type will be notified.
     * @param name name of the data entry
     * @param data original data in the DP will be updated if exist
     */
    virtual void setData(std::string name, boost::any data);
    /**
     * @brief Set the Config object at desired entry. Subscribers of CONFIG type will be notified.
     * @param name name of the data entry
     * @param data original data in the DP will be updated if exist
     */
    virtual void setConfig(std::string name, boost::any data);
    /**
     * @brief Register a subscriber for DATA/CONFIG/ALL
     * @param subscriber 
     */
    virtual void registerEvents(DataPoolSubscriber* subscriber);
    /**
     * @brief Print the list of existing entries
     */
    virtual void printAllEntry();
    /**
     * @brief Print the list of existing entries and current data (Debug only)
     */
    virtual void printAllEntryWithData();  // Debug
    DataPool();
    virtual ~DataPool();
};

}  // namespace DP

#endif  // UTILS_DATAPOOL_  // NOLINT
