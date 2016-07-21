/**
 * @file simple_mission_demo.cpp
 * @brief simple mission example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "attitude_control_demo");
    ros::NodeHandle nh("~");

    double thro, roll_rate, pitch_rate, yaw_rate;
    nh.getParam("throttle", thro);
    nh.getParam("roll_rate", roll_rate);
    nh.getParam("pitch_rate", pitch_rate);
    nh.getParam("yaw_rate", yaw_rate);
    ROS_INFO(" throttle %.2f roll_rate %.2f pitch_rate %.2f yaw_rate %.2f",
		thro, roll_rate, pitch_rate, yaw_rate);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Publisher throttle_pub = nh.advertise<std_msgs::Float64>
            ("/mavros/setpoint_attitude/att_throttle", 10);
    ros::Publisher attitude_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_attitude/attitude", 10);
    ros::Publisher attitude_rate_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_attitude/cmd_vel", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(5.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time last_change = ros::Time::now();

    geometry_msgs::TwistStamped attitude_rate;
    attitude_rate.twist.angular.x = roll_rate;
    attitude_rate.twist.angular.y = pitch_rate;;
    attitude_rate.twist.angular.z = yaw_rate;

    while(ros::ok()){
	std_msgs::Float64 throttle;
	throttle.data = thro; // 0~1
	throttle_pub.publish(throttle);

	geometry_msgs::PoseStamped attitude;
	attitude.pose.orientation.x = roll_rate;
	attitude.pose.orientation.y = pitch_rate;
	attitude.pose.orientation.z = yaw_rate;
	attitude.pose.orientation.w = 1;
	attitude_pub.publish(attitude);

	if (ros::Time::now() - last_change > ros::Duration(3.0)) {
		last_change = ros::Time::now();
		//TODO
	}

//	attitude_rate_pub.publish(attitude_rate);

	ros::spinOnce();
	rate.sleep();
    }

    return 0;
}
