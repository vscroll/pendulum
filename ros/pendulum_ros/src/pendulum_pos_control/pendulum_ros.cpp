#include <ros/ros.h>

#include "pendulum_ctrl_pos.h"

#define CTRL_MODULE_ATT 0
#define CTRL_MODULE_POS 1

int main(int argc, char **argv) {
	ros::init(argc, argv, "pendulum_pos_control");

	ros::NodeHandle nh("~");
/*
	int ctrl_module = 0;
	nh.param<int>("ctrl_module", ctrl_module, CTRL_MODULE_ATT);
*/
	PendulumCtrlPos pendulum_ctrl;
	pendulum_ctrl.open();
	bool is_running = pendulum_ctrl.running();
	try
	{
		ros::Rate rate(10);
		while(ros::ok())
		{
			ros::spinOnce();
			bool flag = pendulum_ctrl.running();
			if (is_running != flag) {
				is_running = flag;
				if (is_running) {
					ROS_INFO("start run pendulum control");
				} else {
					ROS_INFO("stop run pendulum control");
				}
			}
			rate.sleep();
		}

		ROS_INFO("exit ros");
		pendulum_ctrl.close();
		return 1;
	}
	catch (std::exception &ex)
	{
		std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
	}
	return 0;
}
