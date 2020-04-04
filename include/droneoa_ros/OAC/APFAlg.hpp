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
 * Written by Daniel Xu <y365xu@edu.uwaterloo.ca>, April 2020
 */

/**
 * @file APFAlg.hpp
 * @author DroneOA (Daniel Xu)
 * @brief Abstract base class for APF (Artifical Potential Field) Algorithm
 * @version 1.0
 * @date 2020-04
 */

#ifndef INCLUDE_DRONEOA_ROS_OAC_APFALG_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_OAC_APFALG_HPP_  // NOLINT

#include <droneoa_ros/OAC/BaseAlg.hpp>
#include <droneoa_ros/OAC/DataStorage/CircularBuffer.hpp>

class APFAlg : public BaseAlg {
 public:
    explicit APFAlg(CNCInterface *cnc);
    virtual ~APFAlg();

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

    virtual CircularBuffer getCircularBuffer();

 protected:
    /**
     * @brief A circular buffer queue for storing intermediate sensor data
     */
    CircularBuffer CircularBuffer_;
};

#endif  // #define INCLUDE_DRONEOA_ROS_OAC_APFALG_HPP_  // NOLINT