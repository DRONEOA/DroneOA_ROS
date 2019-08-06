/* Copyright (C) DroneOA Group - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Bohan Shi <b34shi@edu.uwaterloo.ca>, August 2019
 */

#include <gtest/gtest.h>
#include "../include/droneoa_ros/Utils.hpp"

TEST(UtilTestSuite, testLocMeter01) {
    GPSPoint newPos = getLocationMeter(GPSPoint(43.484409, -80.53272, 10), 0, 0);
    EXPECT_NEAR(43.484409, newPos.latitude_, 0.0001);
    EXPECT_NEAR(-80.53272, newPos.longitude_, 0.0001);
    EXPECT_NEAR(10, newPos.altitude_, 0.1);
}

TEST(UtilTestSuite, testLocMeter02) {
    GPSPoint newPos = getLocationMeter(GPSPoint(43.484409, -80.53272, 10), 100, 0);
    EXPECT_NEAR(43.485305, newPos.latitude_, 0.0001);
    EXPECT_NEAR(-80.53272, newPos.longitude_, 0.0001);
    EXPECT_NEAR(10, newPos.altitude_, 0.1);
}

TEST(UtilTestSuite, testLocMeter03) {
    GPSPoint newPos = getLocationMeter(GPSPoint(43.484409, -80.53272, 15), 100, 100);
    EXPECT_NEAR(43.485305, newPos.latitude_, 0.0001);
    EXPECT_NEAR(-80.531486, newPos.longitude_, 0.0001);
    EXPECT_NEAR(15, newPos.altitude_, 0.1);
}

TEST(UtilTestSuite, testLocMeter04) {
    GPSPoint newPos = getLocationMeter(GPSPoint(43.484409, -80.53272, 10), 0, 100);
    EXPECT_NEAR(43.484409, newPos.latitude_, 0.0001);
    EXPECT_NEAR(-80.531486, newPos.longitude_, 0.0001);
    EXPECT_NEAR(10, newPos.altitude_, 0.1);
}

TEST(UtilTestSuite, testLocMeter05) {
    GPSPoint newPos = getLocationMeter(GPSPoint(43.484409, -80.53272, 10), 1000, 25000);
    EXPECT_NEAR(43.493392, newPos.latitude_, 0.0001);
    EXPECT_NEAR(-80.223197, newPos.longitude_, 0.0001);
    EXPECT_NEAR(10, newPos.altitude_, 0.1);
}

TEST(UtilTestSuite, testDistMeter01) {
    float dist = getDistanceMeter(GPSPoint(43.12345, -80.34567, 10), GPSPoint(40.78900, -87.234, 10));
    EXPECT_NEAR(809643.5, dist, 0.1);
}

TEST(UtilTestSuite, testDistMeter02) {
    float dist = getDistanceMeter(GPSPoint(43.12345, -80.34567, 10), GPSPoint(43.12345, -80.34567, 10));
    EXPECT_NEAR(0, dist, 0.1);
}

TEST(UtilTestSuite, testBearing01) {
    float bear = getBearing(GPSPoint(43.12345, -80.34567, 10), GPSPoint(40.78900, -87.234, 10));
    EXPECT_NEAR(251.278, bear, 0.01);
}

TEST(UtilTestSuite, testBearing02) {
    float bear = getBearing(GPSPoint(43.12345, -80.34567, 10), GPSPoint(43.12345, -80.34567, 10));
    EXPECT_NEAR(90, bear, 0.01);
}

TEST(UtilTestSuite, testNEHeading01) {
    std::pair<float, float> tmp = getNorthEastDistanceFromHeading(0, 1000);
    EXPECT_NEAR(1000, tmp.first, 0.01);
    EXPECT_NEAR(0, tmp.second, 0.01);
}

TEST(UtilTestSuite, testNEHeading02) {
    std::pair<float, float> tmp = getNorthEastDistanceFromHeading(55, 1000);
    EXPECT_NEAR(573.576, tmp.first, 0.01);
    EXPECT_NEAR(819.152, tmp.second, 0.01);
}

TEST(UtilTestSuite, testNEHeading03) {
    std::pair<float, float> tmp = getNorthEastDistanceFromHeading(90, 1000);
    EXPECT_NEAR(0, tmp.first, 0.01);
    EXPECT_NEAR(1000, tmp.second, 0.01);
}

TEST(UtilTestSuite, testNEHeading04) {
    std::pair<float, float> tmp = getNorthEastDistanceFromHeading(180, 1000);
    EXPECT_NEAR(-1000, tmp.first, 0.01);
    EXPECT_NEAR(0, tmp.second, 0.01);
}

TEST(UtilTestSuite, testNEHeading05) {
    std::pair<float, float> tmp = getNorthEastDistanceFromHeading(270, 1000);
    EXPECT_NEAR(0, tmp.first, 0.01);
    EXPECT_NEAR(-1000, tmp.second, 0.01);
}

TEST(UtilTestSuite, testNEHeading06) {
    std::pair<float, float> tmp = getNorthEastDistanceFromHeading(360, 1000);
    EXPECT_NEAR(1000, tmp.first, 0.01);
    EXPECT_NEAR(0, tmp.second, 0.01);
}

TEST(UtilTestSuite, testNEHeading07) {
    std::pair<float, float> tmp = getNorthEastDistanceFromHeading(-50, 1000);
    EXPECT_NEAR(642.787, tmp.first, 0.01);
    EXPECT_NEAR(-766.044, tmp.second, 0.01);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
