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
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "./test_suite.hpp"

using ::testing::_;
using ::testing::Return;

TEST_F(CommandTest, ChmodCommand_change_mode) {
    EXPECT_CALL(cnc, getMode()).WillOnce(Return(FLT_MODE_GUIDED));
    EXPECT_CALL(cnc, setMode(FLT_MODE_BRAKE)).WillOnce(Return(true));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_CHMOD, FLT_MODE_BRAKE));
}

TEST_F(CommandTest, ChmodCommand_same_mode) {
    EXPECT_CALL(cnc, getMode()).WillOnce(Return(FLT_MODE_GUIDED));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_CHMOD, FLT_MODE_GUIDED));
}

TEST_F(CommandTest, ArmCommand_success) {
    EXPECT_CALL(cnc, setMode(FLT_MODE_GUIDED)).WillOnce(Return(true));
    EXPECT_CALL(cnc, armVehicle()).WillOnce(Return(true));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_ARM, ""));
}

TEST_F(CommandTest, TakeoffCommand_success) {
    EXPECT_CALL(cnc, isArmed()).WillOnce(Return(true));
    EXPECT_CALL(cnc, takeoff(5.01)).WillOnce(Return(true));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_TAKEOFF, "5.01"));
}

TEST_F(CommandTest, TakeoffCommand_not_armed) {
    EXPECT_CALL(cnc, isArmed()).WillOnce(Return(false));
    bool returnValue = Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_TAKEOFF,
    FLT_MODE_STABILIZE));
    EXPECT_EQ(false, returnValue);
}

TEST_F(CommandTest, LandCommand_success) {
    EXPECT_CALL(cnc, land(1)).WillOnce(Return(true));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_LAND, ""));
}

TEST_F(CommandTest, SetMaxVelocityCommand_positive_speed) {
    EXPECT_CALL(cnc, setMaxSpeed(1, 2, 0)).WillOnce(Return(true));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_SET_MAX_VELOCITY, "2"));
}

TEST_F(CommandTest, SetMaxVelocityCommand_negative_speed) {
    EXPECT_CALL(cnc, setMaxSpeed(1, 0, 0)).WillOnce(Return(true));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_SET_MAX_VELOCITY, "-0.5"));
}

TEST_F(CommandTest, SetYawCommand_success) {
    EXPECT_CALL(cnc, setYaw(30, false, false)).WillOnce(Return(true));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_SET_YAW, "30"));
}

TEST_F(CommandTest, GotoRelativeCommand_success) {
    EXPECT_CALL(cnc, getRelativeAltitude()).WillOnce(Return(5));
    EXPECT_CALL(cnc, gotoRelative(5, -2, 10, false, false));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_GOTO_RELATIVE, "5 -2 10"));
}

TEST_F(CommandTest, GotoGlobalCommand_success) {
    EXPECT_CALL(cnc, getRelativeAltitude()).WillOnce(Return(5));
    EXPECT_CALL(cnc, gotoGlobal(5, -2, 10, false));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_GOTO_GLOBAL, "5 -2 10"));
}

TEST_F(CommandTest, GotoHeadingCommand_success) {
    EXPECT_CALL(cnc, getRelativeAltitude()).WillOnce(Return(10));
    EXPECT_CALL(cnc, setYaw(30, false, false)).WillOnce(Return(true));
    EXPECT_CALL(cnc, gotoHeading(30, 1.0, 5, false)).WillOnce(Return(true));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_GOTO_HEADING, "30 1.0 5"));
}

TEST_F(CommandTest, GotoHeadingCommand_yaw_set_failed) {
    EXPECT_CALL(cnc, getRelativeAltitude()).WillOnce(Return(10));
    EXPECT_CALL(cnc, setYaw(30, false, false)).WillOnce(Return(false));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_GOTO_HEADING, "30 1.0 5"));
}

TEST_F(CommandTest, ClimbCommand_success) {
    EXPECT_CALL(cnc, getRelativeAltitude()).WillOnce(Return(5));
    mavros_msgs::VFR_HUD msgs{};
    msgs.heading = 10;
    EXPECT_CALL(cnc, getHUDData()).WillOnce(Return(msgs));
    EXPECT_CALL(cnc, gotoHeading(10, 0.0f, 15, false)).WillOnce(Return(true));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_CLIMB, "10"));
}

TEST_F(CommandTest, DescendCommand_success) {
    EXPECT_CALL(cnc, getRelativeAltitude()).WillOnce(Return(10));
    mavros_msgs::VFR_HUD msgs{};
    msgs.heading = 10;
    EXPECT_CALL(cnc, getHUDData()).WillOnce(Return(msgs));
    EXPECT_CALL(cnc, gotoHeading(10, 0.0f, 5, false)).WillOnce(Return(true));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_DESCEND, "5"));
}
