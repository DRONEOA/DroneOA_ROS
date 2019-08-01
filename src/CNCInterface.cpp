#include <droneoa_ros/CNCInterface.hpp>
#include <cstdlib>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

CNCInterface::CNCInterface() {

}

CNCInterface::~CNCInterface() {

}

void CNCInterface::init(ros::NodeHandle nh) {
    n = nh;
}

// Mode Control
bool CNCInterface::setMode(std::string modeName) {
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = modeName;
    if(cl.call(srv_setMode)){
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
        return true;
    }else{
        ROS_ERROR("Failed SetMode");
        return false;
    }
}

// Safety
bool CNCInterface::armVehicle() {
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok %d", srv.response.success);
        return true;
    }else{
        ROS_ERROR("Failed arming or disarming");
        return false;
    }
}

// Guided Flight Control
bool CNCInterface::takeoff(int targetAltitude) {
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = targetAltitude;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
        return true;
    }else{
        ROS_ERROR("Failed Takeoff");
        return false;
    }
}

bool CNCInterface::land(int fromAltitude) {
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = fromAltitude;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
        return true;
    }else{
        ROS_ERROR("Failed Land");
        return false;
    }
}