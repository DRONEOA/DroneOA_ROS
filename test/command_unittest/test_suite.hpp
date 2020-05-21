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
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, April 2020
 */

#ifndef UT_CNCUNITTEST_TEST_SUITE_HPP  // NOLINT
#define UT_CNCUNITTEST_TEST_SUITE_HPP  // NOLINT

#include <gtest/gtest.h>

#define private public
#define protected public
#include "../../include/droneoa_ros/OAC/Command.hpp"
#undef protected
#undef private

#include "../mocks/mock_cnc.hpp"

class CommandTest: public ::testing::Test {
 public:
    CommandTest();
    void SetUp() override;
    void TearDown() override;

    // Mock instances
    CNC::CNCMock cnc;
};

#endif  // UT_CNCUNITTEST_TEST_SUITE_HPP  // NOLINT
