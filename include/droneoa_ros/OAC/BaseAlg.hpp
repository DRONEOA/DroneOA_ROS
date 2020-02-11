/* Copyright (C) 2019 DroneOA Group - All Rights Reserved
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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

/**
 * @file BaseAlg.hpp
 * @author DroneOA (Bohan Shi)
 * @brief Abstract base class for algorithms
 * @version 1.0
 * @date 2019-08
 */

#ifndef INCLUDE_DRONEOA_ROS_OAC_BASEALG_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_OAC_BASEALG_HPP_  // NOLINT

#include <utility>
#include <string>
#include <vector>
#include <droneoa_ros/CNCInterface.hpp>
#include <droneoa_ros/OAC/Command.hpp>

class BaseAlg {
 public:
    explicit BaseAlg(CNCInterface *cnc);
    virtual ~BaseAlg();

    /**
     * @brief Init the algorithm instance
     * Seperated from constructor due to the planned restart feature
     * @param cnc pointer to the Command And Control interface
     */
    virtual void init(CNCInterface *cnc);  // For restart
    /**
     * @brief Collect Data From Sensor And FCU Interface
     * @return false if errorif execute with error or precondition not satisfied
     */
    virtual bool collect() = 0;
    /**
     * @brief Plan Command Queue For Current Frame / Cycle. Populate CMDQueue_ and DATAQueue_
     * @return false when get around is impossible (can not fly safely with this algorithm)
     */
    virtual bool plan() = 0;

    virtual CommandQueue getCommandQueue();
    virtual DataQueue getDataQueue();

 protected:
    CNCInterface *cnc_;
    /**
     * @brief A vector of CommandLine which is a pair of CMD_QUEUE_TYPES and std::string
     */
    CommandQueue CMDQueue_;
    /**
     * @brief A vector of DataLine which is a pair of DATA_QUEUE_TYPES and std::string
     */
    DataQueue DATAQueue_;
};

#endif  // INCLUDE_DRONEOA_ROS_OAC_BASEALG_HPP_  // NOLINT
