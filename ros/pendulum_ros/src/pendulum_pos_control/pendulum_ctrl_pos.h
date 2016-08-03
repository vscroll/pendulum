#ifndef __PENDULUM_CTRL_POS_H__
#define __PENDULUM_CTRL_POS_H__

#include <ros/ros.h>

#include "../pendulum_common/pendulum_ctrl_base.h"

class PendulumCtrlPos: public PendulumCtrlBase {

public:
	PendulumCtrlPos();
	~PendulumCtrlPos();

public:
	virtual void ctrl_thread();

private:
	ros::Publisher _local_pos_pub;
	uint64_t _last_time_ms;
};

#endif
