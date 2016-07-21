#ifndef __PENDULUM_CTRL_ATT_H__
#define __PENDULUM_CTRL_ATT_H__

#include <ros/ros.h>

#include "pendulum_ctrl_base.h"

class PendulumCtrlAtt: public PendulumCtrlBase {

public:
	PendulumCtrlAtt();
	~PendulumCtrlAtt();

public:
	virtual void ctrl_thread();

private:
	ros::Publisher _throttle_pub;
	ros::Publisher _attitude_pub;
};

#endif
