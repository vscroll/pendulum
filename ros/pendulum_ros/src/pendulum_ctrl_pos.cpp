#include "pendulum_ctrl_pos.h"
#include "pendulum_dynamic.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

PendulumCtrlPos::PendulumCtrlPos() {
	_local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
	_last_time_ms = 0;
}

PendulumCtrlPos::~PendulumCtrlPos() {

}

void PendulumCtrlPos::ctrl_thread() {
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
			//_pendulum_output_x = _pose.position.x - _vehicle_pose_local.position.x;
			//_pendulum_output_y = _pose.position.y - _vehicle_pose_local.position.y;

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
#if 0
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
#endif

			if (_last_time_ms == 0) {
				_last_time_ms = ros::Time::now().toSec() * 1000;
			} else {
				uint64_t dt_ms = ros::Time::now().toSec() * 1000 - _last_time_ms;
				_last_time_ms = ros::Time::now().toSec() * 1000;
				//ROS_INFO("secs %f dt_ms %ld", (dt_ms / 1000.0), dt_ms);

				double x = _vehicle_pose_local.position.x + 0.5 * vehicle_vel_acc_x * (dt_ms / 1000.0) * (dt_ms / 1000.0);
				double y = _vehicle_pose_local.position.y + 0.5 * vehicle_vel_acc_y * (dt_ms / 1000.0) * (dt_ms / 1000.0);
				//ROS_INFO("accx %f accy %f x %f y %f", vehicle_vel_acc_x, vehicle_vel_acc_y, x, y);

				if (_local_pos_pub) {
					// enu->ned x = x y = -y z = -z
					auto p_ned = mavros::ftf::transform_frame_enu_ned(Eigen::Vector3d(x, -y, -_vehicle_pose_local.position.z));
					// keep yaw angle
					Eigen::Quaterniond q_enu = mavros::ftf::quaternion_from_rpy(0, 0, 0);

					auto q_ned = mavros::ftf::transform_orientation_enu_ned(
						mavros::ftf::transform_orientation_aircraft_baselink(q_enu));

					geometry_msgs::PoseStamped pose;
					pose.pose.position.x = p_ned[0];
					pose.pose.position.y = p_ned[1];
					pose.pose.position.z = p_ned[2];
					pose.pose.orientation.x = q_ned.x();
					pose.pose.orientation.y = q_ned.y();
					pose.pose.orientation.z = q_ned.z();
					pose.pose.orientation.w = q_ned.w();

					_local_pos_pub.publish(pose);
				}
			}
		}

		_rate.sleep();
	}

	ROS_INFO("exit ctrl_thread");
	_started = false;
}
