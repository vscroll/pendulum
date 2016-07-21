#ifndef __PENDULUM_CTRL_BASE_H__
#define __PENDULUM_CTRL_BASE_H__

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <pendulum_ros/PendulumConfig.h>
#include <fmaros_msgs/PendulumPose.h>
#include <fmaros_msgs/VehiclePose.h>

#include "PID.h"

class PendulumCtrlBase {

public:
	PendulumCtrlBase();
	~PendulumCtrlBase();

public:
	bool open();
	bool running();
	void sleep();
	void close();

	virtual void ctrl_thread() = 0;
protected:
	void reset_pose();

private:
	void start();
	void stop();

private:
	void armed();
	void disarmed();
	void takeoff();
	void set_posctl();
	void set_offboard();
	void set_cmd_sequence();

private:
	void cfg_pid_callback(pendulum_ros::PendulumConfig &config, uint32_t level);

	static void* thread_run(void* argument);

	void pendulum_pose_callback(const fmaros_msgs::PendulumPose::ConstPtr& msg);
	void vehicle_pose_callback(const fmaros_msgs::VehiclePose::ConstPtr& msg);

protected:
	ros::Rate _rate;
	ros::NodeHandle _nh;

	dynamic_reconfigure::Server<pendulum_ros::PendulumConfig> _cfg_server;
	CVG_BlockDiagram::PID _pendulum_x_pid;
	CVG_BlockDiagram::PID _pendulum_y_pid;

	bool _started;
	pthread_t _thread;

	ros::Subscriber _pendulum_sub;
	ros::Subscriber _vehicle_sub;

	ros::ServiceClient _arming_client;
	ros::ServiceClient _set_mode_client;

	static const double DEFAULT_PENDULUM_L = 0.6;
	static const double DEFAULT_PENDULUM_X_P = 0.5;
	static const double DEFAULT_PENDULUM_X_I = 0.0;
	static const double DEFAULT_PENDULUM_X_D = 0.0;
	static const double DEFAULT_PENDULUM_Y_P = 0.5;
	static const double DEFAULT_PENDULUM_Y_I = 0.0;
	static const double DEFAULT_PENDULUM_Y_D = 0.0;

	double _pendulum_l;
	fmaros_msgs::PendulumPose _pose_local;
	fmaros_msgs::PendulumPose _pose;

	double _pendulum_output_x;
	double _pendulum_output_y;

	bool _reset_pose;

	static const int PENDULUM_CMD_DISABLE = 0;
	static const int PENDULUM_CMD_START = 1;
	static const int PENDULUM_CMD_STOP = 2;

	static const int VEHICLE_CMD_DISABLE = 0;
	static const int VEHICLE_CMD_ARMED = 1;
	static const int VEHICLE_CMD_DISARMED = 2;
	static const int VEHICLE_CMD_TAKEOFF = 3;
	static const int VEHICLE_CMD_POSCTL = 4;
	static const int VEHICLE_CMD_OFFBOARD = 5;
	static const int VEHICLE_CMD_SEQUENCE = 6;

};

#endif
