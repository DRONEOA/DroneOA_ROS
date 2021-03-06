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

#ifndef OAC_BASEALG_HPP_  // NOLINT
#define OAC_BASEALG_HPP_  // NOLINT

#include <utility>
#include <string>
#include <vector>
#include <droneoa_ros/HWI/interface/CNCInterface.hpp>
#include <droneoa_ros/PDN.hpp>
#include <droneoa_ros/OAC/Command.hpp>

namespace OAC {
class BaseAlg {
 public:
    explicit BaseAlg(CNC::CNCInterface *cnc);
    virtual ~BaseAlg();

    /**
     * @brief Init the algorithm instance
     * Seperated from constructor due to the planned restart feature
     * @param cnc pointer to the Command And Control interface
     */
    virtual void init(CNC::CNCInterface *cnc);  // For restart
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

    virtual Command::CommandQueue getCommandQueue();
    virtual Command::DataQueue getDataQueue();

 protected:
    CNC::CNCInterface *mpCNC;
    /**
     * @brief A vector of CommandLine which is a pair of CMD_QUEUE_TYPES and std::string
     */
    Command::CommandQueue CMDQueue_;
    /**
     * @brief A vector of DataLine which is a pair of DATA_QUEUE_TYPES and std::string
     */
    Command::DataQueue DATAQueue_;
};

}  // namespace OAC

#endif  // OAC_BASEALG_HPP_  // NOLINT
