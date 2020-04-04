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
 */

#include <gtest/gtest.h>
#include <../include/droneoa_ros/CNCInterface.hpp>
#include "ros/ros.h"
#include <mavros_msgs/CommandTOL.h>
#include <iostream>

#include "gmock/gmock.h"

// bool takeoff(float);

class MockNodeHandle: public ros::NodeHandle {
    public:
        MockNodeHandle() {}
        MOCK_METHOD1(call, bool(mavros_msgs::CommandTOL srv_takeoff));
};

// class MockServiceClient: public ros::ServiceClient {
//     public:

// }

// TODO not working
class MockRate {
    int32_t rate;
    public:
        MockRate(int32_t rate) : rate(rate) {}
};

// class MockRos 


TEST(CNCTestSuite, takeoffSuccess) {
    // TODO mock the call() method of ServiceClient
    // MockCNCInterface CNCInterface;
    // gmock
    // CNCInterface 

    // ros::init(b, a, "mavros_takeoff");
    // std::cout << "..asdfadsf" << std::endl;
    // MockNodeHandle n;
    // EXPECT_CALL(n, call).Times(1);
    std::cout << ",,asdfadsf" << std::endl;
    MockRate r(10);
    std::cout << "//asdfadsf" << std::endl;

    // CNCInterface cnc;
    // std::cout << "--asdfadsf" << std::endl;
    // cnc.init(n, r);
    // std::cout << "==asdfadsf" << std::endl;

    // need to mock n.serviceClient
    // ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");


    // bool result = cnc.takeoff(40.0);
    // EXPECT_TRUE(result); 
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
