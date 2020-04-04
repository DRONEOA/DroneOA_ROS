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
 * @file CircularBuffer.hpp
 * @author DroneOA (Daniel Xu)
 * @brief A circular buffer data structure for storing intermediate sensor data
 * @version 1.0
 * @date 2020-04
 */

#ifndef INCLUDE_DRONEOA_ROS_OAC_CRCLR_BFFR_HPP_  // NOLINT
#define INCLUDE_DRONEOA_ROS_OAC_CRCLR_BFFR_HPP_  // NOLINT


class CircularBuffer {
 public:
    explicit CircularBuffer(int numElements);
    virtual ~CircularBuffer();

    /**
     * @brief push new data into this buffer
     * @param new_data data to be pushed
     */
    void push(float* new_data);  // PUsh a new layer of data
    /**
     * @brief Get specified element from buffer
     * @return float[] data in index position of this buffer
     */
    float* getElement(int position);
    /**
     * @brief Clear all data in buffer.
     */
    void clear();

 private:
    int number_of_elements;  // buffer size
};

#endif  // #define INCLUDE_DRONEOA_ROS_OAC_CRCLR_BFFR_HPP_  // NOLINT