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

TEST_F(CommandTest, ChmodCommand) {
    EXPECT_CALL(cnc, getMode()).WillOnce(Return(FLT_MODE_GUIDED));
    EXPECT_CALL(cnc, setMode(FLT_MODE_BRAKE)).WillOnce(Return(true));
    Command::parseCMD(&cnc, Command::CommandLine(Command::CMD_QUEUE_TYPES::CMD_CHMOD, FLT_MODE_BRAKE));
}
