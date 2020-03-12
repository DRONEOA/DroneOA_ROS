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
 * Written by Zhiqing Hu <clair.hu.1@edu.uwaterloo.ca>, March 2020
 */

#include <gtest/gtest.h>
#include <../include/droneoa_ros/CNCInterface.hpp>
#include "ros/ros.h"
#include "gmock/gmock.h"
// #include "mock_objects/"

class MockCNCInterface: public CNCInterface {
    public:
        MOCK_METHOD(bool, takeoff, (float targetAltitude), (override));
};


TEST(CNCTestSuite, takeoffSuccess) {
    // TODO mock the call() method of ServiceClient
    MockCNCInterface CNCInterface;
    // gmock
    
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    // return RUN_ALL_TESTS();
}
