#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>

#include <droneoa_ros/PackageManager/PMCommandParser.hpp>

/********************************************************
 * Method
 *      Both main node and PM accept PM inputs
 *          main node forward to PM
 *      Note: PM need a list of node
 *      Note: need a config for each addon, like, whether to start with main
 *      PS: we may want main node to broadcast all unacceped console inputs
 *      PS: make main node module help command only, like when type help. Maybe only notify, not main node command
 *      PS: other node free to pick up if matched, no need to inform main node
 */

int main(int argc, char **argv){
    // Set up ROS.
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Rate r(10);

    std::string line;
    PM::CommandParser mPM;
    // Main loop.
    while (n.ok() && std::getline(std::cin, line)) {
        mPM.parseInput(line);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end main()