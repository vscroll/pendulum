#include "pendulum_ctrl_att.h"
#include "pendulum_dynamic.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

PendulumCtrlAtt::PendulumCtrlAtt() {
	_throttle_pub = _nh.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 10);
//	_attitude_pub = _nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 10);
	_attitude_pub = _nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel", 10);
}

PendulumCtrlAtt::~PendulumCtrlAtt() {

}

void PendulumCtrlAtt::ctrl_thread() {
	ROS_INFO("%s(%d): %s", __FILE__, __LINE__, __FUNCTION__);
	while(ros::ok()) {
		ros::spinOnce();
		if (!_started) {
			_rate.sleep();
			continue;
		}

		if (_pose.header.seq == _pose_local.header.seq) {
			ROS_INFO("no pose data");
			_rate.sleep();
			continue;
		}

		_pose = _pose_local;
		if (_pose.header.seq <= 0) {
			_rate.sleep();
			continue;
		}

		if (_reset_pose) {
			ROS_INFO("reset pose");
			_reset_pose = false;
			_pendulum_x_pid.reset();
			_pendulum_y_pid.reset();

			_pendulum_x_pid.setReference(_pose.position.x);
			_pendulum_y_pid.setReference(_pose.position.y);
		} else {
			_pendulum_x_pid.setFeedback(_pose.position.x);
			_pendulum_y_pid.setFeedback(_pose.position.y);

			_pendulum_output_x = _pendulum_x_pid.getOutput();
			_pendulum_output_y = _pendulum_y_pid.getOutput();

			double vehicle_vel_acc_x = 0.0;
			double vehicle_vel_acc_y = 0.0;
			PendulumDynamic::formula_12(_pendulum_l,
								_pendulum_output_x,
								_pendulum_output_y,
								_pose.velocity,
								_pose.vel_acc,
								&vehicle_vel_acc_x,
								&vehicle_vel_acc_y);
			//ROS_INFO("formula 12:%f %f", vehicle_vel_acc_x, vehicle_vel_acc_y);

			double angle_x = 0.0;
			double angle_y = 0.0;
			double a = 0;
			PendulumDynamic::formula_5_2(vehicle_vel_acc_x, vehicle_vel_acc_y,
								&angle_x, &angle_y, &a);
			//ROS_INFO("formula 5:%f %f %f %f", angle_x, angle_y, a, a/(2*PendulumDynamic::g));

			double vehicle_rate_x = 0.0;
			double vehicle_rate_y = 0.0;
			PendulumDynamic::formula_7(angle_x, angle_y,
									angle_x, angle_y,
									&vehicle_rate_x, &vehicle_rate_y);
			//ROS_INFO("formula 7:%f %f", vehicle_rate_x, vehicle_rate_y);

/*
			ROS_INFO("result:%d    %f %f %f %f    %f %f %f %f    %f %f    %f %f %f %f    %f %f",
				_pose.header.seq,
				_pose.position.x, _pose.position.y, _pendulum_output_x, _pendulum_output_y,
				_pose.velocity.x, _pose.velocity.y, _pose.vel_acc.x, _pose.vel_acc.y,
				vehicle_vel_acc_x, vehicle_vel_acc_y,
				angle_x, angle_y, a, a/(2*PendulumDynamic::g),
				vehicle_rate_x, vehicle_rate_y);
*/
			if (_throttle_pub) {
				std_msgs::Float64 throttle;
				throttle.data = 0.49000;//a/(2*PendulumDynamic::g); // 0~1
				_throttle_pub.publish(throttle);
			}

			if (_attitude_pub) {
				geometry_msgs::TwistStamped attitude;
				attitude.twist.angular.x = 0;//vehicle_rate_x;
				attitude.twist.angular.y = 0;//vehicle_rate_y;
				attitude.twist.angular.z = 0.1;
				_attitude_pub.publish(attitude);
			}
		}

		_rate.sleep();
	}

	ROS_INFO("exit ctrl_thread");
	_started = false;
}
