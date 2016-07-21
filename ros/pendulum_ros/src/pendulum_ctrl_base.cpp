#include "pendulum_ctrl_base.h"
#include "pendulum_dynamic.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

PendulumCtrlBase::PendulumCtrlBase()
	:_nh( "~" ),
	_started(false),
	_rate(200),
	_pendulum_l(DEFAULT_PENDULUM_L),
	_reset_pose(false) {
	
	_pendulum_sub = _nh.subscribe("/FMA/Pendulum/Pose", 100, &PendulumCtrlBase::pendulum_pose_callback, this);

	_arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	_set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

	_pendulum_x_pid.setGains(DEFAULT_PENDULUM_X_P, DEFAULT_PENDULUM_X_I, DEFAULT_PENDULUM_X_D);
	_pendulum_x_pid.setGains(DEFAULT_PENDULUM_Y_P, DEFAULT_PENDULUM_Y_I, DEFAULT_PENDULUM_Y_D);

	dynamic_reconfigure::Server<pendulum_ros::PendulumConfig>::CallbackType f;
	f = boost::bind(&PendulumCtrlBase::cfg_pid_callback, this, _1, _2);
	_cfg_server.setCallback(f);

	reset_pose();
}

PendulumCtrlBase::~PendulumCtrlBase() {

}

void PendulumCtrlBase::cfg_pid_callback(pendulum_ros::PendulumConfig &config, uint32_t level) {

	ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %d %d",
			config.pendulum_l,
			config.pendulum_x_p,
			config.pendulum_x_i, 
			config.pendulum_x_d,
			config.pendulum_y_p,
			config.pendulum_y_i, 
			config.pendulum_y_d,
			config.pendulum_cmd,
			config.vehicle_cmd
			);

	_pendulum_l = config.pendulum_l;
	_pendulum_x_pid.setGains(config.pendulum_x_p, config.pendulum_x_i, config.pendulum_x_d);
	_pendulum_y_pid.setGains(config.pendulum_y_p, config.pendulum_y_i, config.pendulum_y_d);

	if (config.pendulum_cmd == PENDULUM_CMD_START) {
		_reset_pose = true;
		reset_pose();

		start();
	} else if (config.pendulum_cmd == PENDULUM_CMD_STOP) {
		stop();
	} else {
	}

	if (config.vehicle_cmd == VEHICLE_CMD_ARMED) {
		armed();
	} else if (config.vehicle_cmd == VEHICLE_CMD_DISARMED) {
		disarmed();
	} else if (config.vehicle_cmd == VEHICLE_CMD_TAKEOFF) {
		takeoff();
	} else if (config.vehicle_cmd == VEHICLE_CMD_POSCTL) {
		set_posctl();
	} else if (config.vehicle_cmd == VEHICLE_CMD_OFFBOARD) {
		set_offboard();
	} else if (config.vehicle_cmd == VEHICLE_CMD_SEQUENCE) {
		set_cmd_sequence();
	} else {
	}
}

void PendulumCtrlBase::reset_pose() {
	memset(&_pose.header.stamp, 0, sizeof(_pose.header.stamp));
	_pose.header.seq = 0;
}

bool PendulumCtrlBase::open() {
	ROS_INFO("open");
	pthread_create(&_thread, NULL, &PendulumCtrlBase::thread_run, this);

}

void PendulumCtrlBase::close() {
	ROS_INFO("close");
	if (_started) {
		stop();
	}
}

void* PendulumCtrlBase::thread_run(void* argument) {
	((PendulumCtrlBase*)argument)->ctrl_thread();
	return NULL;
}

void PendulumCtrlBase::start() {
	ROS_INFO("start");
	_started = true;
}

void PendulumCtrlBase::stop() {
	ROS_INFO("stop");
	_started = false;
}

bool PendulumCtrlBase::running() {
	return _started;
}

void PendulumCtrlBase::armed() {
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	if (_arming_client) {
		if(_arming_client.call(arm_cmd) &&
				arm_cmd.response.success) {
			ROS_INFO("Vehicle armed");
		}
	}
}

void PendulumCtrlBase::disarmed() {
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = false;
	if (_arming_client) {
		if (_arming_client.call(arm_cmd) &&
				arm_cmd.response.success) {
			ROS_INFO("Vehicle armed");
		}
	}
}

void PendulumCtrlBase::takeoff() {
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
	if (_set_mode_client) {
		if (_set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.success){
			ROS_INFO("takeoff enabled");
		}
	}
}

void PendulumCtrlBase::set_posctl() {
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "POSCTL";
	if (_set_mode_client) {
		if (_set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.success){
			ROS_INFO("posctl enabled");
		}
	}
}

void PendulumCtrlBase::set_offboard() {
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	if (_set_mode_client) {
		if (_set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.success){
			ROS_INFO("Offboard enabled");
		}
	}
}

void PendulumCtrlBase::set_cmd_sequence() {
	ros::Rate rate(1);
	armed();
	rate.sleep();
	takeoff();
	rate.sleep();
	rate.sleep();
	rate.sleep();
	rate.sleep();

	_reset_pose = true;
	reset_pose();
	start();
	rate.sleep();

	set_offboard();
}
void PendulumCtrlBase::pendulum_pose_callback(const fmaros_msgs::PendulumPose::ConstPtr& msg) {
/*
	_pose_local.header = msg->header;
	_pose_local.position = msg->position;
	_pose_local.velocity = msg->velocity;
	_pose_local.vel_acc = msg->vel_acc;
*/
	_pose_local = *msg;

/*
	ROS_INFO("Pendulum Pose: %f %f %f %f %f %f %f %f %f",
		_pose_local.position.x, _pose_local.position.y, _pose_local.position.z,
		_pose_local.velocity.x, _pose_local.velocity.y, _pose_local.velocity.z,
		_pose_local.vel_acc.x, _pose_local.vel_acc.y, _pose_local.vel_acc.z);
*/
}

void PendulumCtrlBase::vehicle_pose_callback(const fmaros_msgs::VehiclePose::ConstPtr& msg) {

}

