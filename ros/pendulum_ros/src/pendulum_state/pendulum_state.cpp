#include <ros/ros.h>

#include <fmaros_msgs/VehiclePose.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <fstream>

std::ofstream _vehicle_pose_file;
std::ofstream _throttle_file;

mavros_msgs::State _current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    _current_state = *msg;
	ROS_INFO("%s", _current_state.mode.c_str());
}

fmaros_msgs::VehiclePose _current_pose;
void vehicle_pose_cb(const fmaros_msgs::VehiclePose::ConstPtr& msg){
    _current_pose = *msg;
	if (_current_state.mode == "OFFBOARD") {
		_vehicle_pose_file << _current_pose.position.z << std::endl;
	}
}

std_msgs::Float64 _current_throttle;
void throttle_cb(const std_msgs::Float64::ConstPtr& msg){
    _current_throttle = *msg;
	if (_current_state.mode == "OFFBOARD") {
		_throttle_file << _current_throttle.data << std::endl;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pendulum_state_node");

	ros::NodeHandle nh("~");
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",
								10, state_cb);
    ros::Subscriber vehicle_sub = nh.subscribe<fmaros_msgs::VehiclePose>("/FMA/Vehicle/Pose",
								10, vehicle_pose_cb);
    ros::Subscriber throttle_pub = nh.subscribe<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle",
								10, throttle_cb);

	_vehicle_pose_file.open ("./vehicle_pose.txt");
	_vehicle_pose_file << "Writing vehicle_pose to a file.\n";

	_throttle_file.open ("./throttle.txt");
	_throttle_file << "Writing throttle to a file.\n";
	try
	{
		ros::Rate rate(10);
		while(ros::ok())
		{
			ros::spinOnce();
			
			rate.sleep();
		}
		
		_vehicle_pose_file.close();
		_throttle_file.close();
		return 1;
	}
	catch (std::exception &ex)
	{
		std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
	}
	return 0;
}
