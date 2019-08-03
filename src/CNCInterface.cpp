#include <droneoa_ros/CNCInterface.hpp>
#include <cstdlib>
#include <iostream>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

CNCInterface::CNCInterface() {

}

CNCInterface::~CNCInterface() {

}

void CNCInterface::init(ros::NodeHandle nh, ros::Rate r) {
    n = nh;
    r_ = r;
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

    // TODO: Should we move those to constructor ?
    thread_watch_state_ = new boost::thread(boost::bind(&CNCInterface::watchStateThread, this));
    thread_watch_GPSFix_ = new boost::thread(boost::bind(&CNCInterface::watchGPSFixThread, this));

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

// Navigation
bool CNCInterface::setHome(float targetLatitude, float targetLongitude, float targetAltitude) {
    ros::ServiceClient setHome_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/set_home");
    mavros_msgs::CommandTOL srv_setHome;
    srv_setHome.request.altitude = targetAltitude;
    srv_setHome.request.latitude = targetLatitude;
    srv_setHome.request.longitude = targetLongitude;
    if(setHome_cl.call(srv_setHome)){
        ROS_INFO("srv_setHome send ok %d", srv_setHome.response.success);
        return true;
    }else{
        ROS_ERROR("Failed Land");
        return false;
    }
}

// Mission
bool CNCInterface::pushWaypoints(float x_lat, float y_long, float z_alt, uint8_t isCurrent, uint16_t command) {
    std::cout << "+pushWaypoints::Current Mode: " << getMode() << std::endl;

    ros::ServiceClient pushWP_cl = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    mavros_msgs::WaypointPush wp_push_srv; // List of Waypoints
    mavros_msgs::Waypoint wp;
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = command;
    wp.is_current     = isCurrent;
    wp.autocontinue   = true;
    wp.x_lat          = x_lat;
    wp.y_long         = y_long;
    wp.z_alt          = z_alt;
    wp_push_srv.request.waypoints.push_back(wp);
    // Send WPs to Vehicle
    if (pushWP_cl.call(wp_push_srv)) {
        ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
        return true;
    } else {
        ROS_ERROR("Send waypoints FAILED.");
        return false;
    }
}

bool CNCInterface::clearWaypoint() {
    ros::ServiceClient clearWP_cl = n.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
    mavros_msgs::WaypointClear wp_clear_srv;
    if (clearWP_cl.call(wp_clear_srv)) {
        ROS_INFO("Clear waypoints ok: %d", wp_clear_srv.response.success);
        return true;
    } else {
        ROS_ERROR("Clear waypoints FAILED.");
        return false;
    }
}

// Callback
/* State */
void CNCInterface::state_callback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
/* GPS Fix */
void CNCInterface::gpsFix_callback(const sensor_msgs::NavSatFixConstPtr& msg) {
    current_gps_fix_ = *msg;
}

// Threads
/* State */
void CNCInterface::watchStateThread() {
  auto state_sub =
        n.subscribe<mavros_msgs::State>("mavros/state", 1, boost::bind(&CNCInterface::state_callback, this, _1));

  while (ros::ok())
  {
    ros::spinOnce();
    r_.sleep();
  }
}
/* GPS Fix */
void CNCInterface::watchGPSFixThread() {
  auto gpsFix_sub =
        n.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, boost::bind(&CNCInterface::gpsFix_callback, this, _1));

  while (ros::ok())
  {
    ros::spinOnce();
    r_.sleep();
  }
}

// Status
/* State */
bool CNCInterface::isConnected() {
    return current_state.connected;
}

bool CNCInterface::isArmed() {
    return current_state.armed;
}

bool CNCInterface::isGuided() {
    return current_state.guided;
}

std::string CNCInterface::getMode() {
    return current_state.mode;
}

uint8_t CNCInterface::getSysStatus() {
    return current_state.system_status;
}

/* GPS Fix */
GPSPoint CNCInterface::getCurrentGPSPoint() {
    return GPSPoint(current_gps_fix_.latitude, current_gps_fix_.longitude, current_gps_fix_.altitude);
}

// User Simple Functions
bool CNCInterface::gotoGlobal(float x_lat, float y_long, float z_alt) {
    // TODO: check Guided mode
    bool rel = false;
    rel = clearWaypoint();
    if (!rel) {
        return rel;
    }
    rel = pushWaypoints(x_lat, y_long, z_alt);
    return rel;
}