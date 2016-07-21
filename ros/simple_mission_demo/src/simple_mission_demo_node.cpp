/**
 * @file simple_mission_demo.cpp
 * @brief simple mission example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>
#include <nav_msgs/Odometry.h>
#include <mavros/gps_conversions.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

bool local_pose_valid = false;
geometry_msgs::PoseStamped local_pose;
void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose = *msg;
    local_pose_valid = true;
}

bool global_position_valid = false;
nav_msgs::Odometry global;
void global_position_cb(const nav_msgs::Odometry::ConstPtr& msg){
    global = *msg;
    global_position_valid = true;
}

bool home_valid = false;
mavros_msgs::HomePosition home;
void home_cb(const mavros_msgs::HomePosition::ConstPtr& msg){
    home = *msg;
    home_valid = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_mission_demo");
    ros::NodeHandle nh("~");

    bool use_home;
    double lat, lon, alt;
    nh.getParam("waypoint_lat", lat);
    nh.getParam("waypoint_lon", lon);
    nh.getParam("waypoint_alt", alt);
    nh.getParam("use_home", use_home);
    ROS_INFO(" lat lon alt use_home %.7f %.7f %.1f %d", lat, lon, alt, use_home);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::Subscriber local_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_pose_cb);
    ros::Subscriber global_sub = nh.subscribe<nav_msgs::Odometry>
            ("/mavros/global_position/local", 10, global_position_cb);
    ros::Subscriber home_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("/mavros/home", 10, home_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
	if (local_pose_valid && global_position_valid) {
		// Local position (UTM) calculation
		double northing, easting;
		std::string zone;
		if (use_home && home_valid) {
			UTM::LLtoUTM(home.latitude, home.longitude, northing, easting, zone);
		} else {
			UTM::LLtoUTM(lat, lon, northing, easting, zone);
		}

		double dx,dy;
		dx = easting - global.pose.pose.position.x;
		dy = northing - global.pose.pose.position.y;

		ROS_INFO(" UTM home(%.2f %.2f) local(%.2f % .2f) diff(%.lf %.lf)", easting, northing, global.pose.pose.position.x , global.pose.pose.position.y, dx, dy);

		if (sqrtf(dx * dx + dy * dy) > 500.0) {
			ROS_INFO(" waypoint too far, reject simple mission");
		} else {
			geometry_msgs::PoseStamped pose;
			pose.pose.position.x = local_pose.pose.position.x + dx;
			pose.pose.position.y = local_pose.pose.position.y + dy;
			pose.pose.position.z = alt;
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 0.0;
			local_pos_pub.publish(pose);
		}
	}

	ros::spinOnce();
	rate.sleep();
    }

    return 0;
}
