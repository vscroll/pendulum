#! /bin/bash                                                                                                                                       

#run in dir catkin_ws

STACK_PATH=./
source /opt/ros/indigo/setup.bash
source ${STACK_PATH}/devel/setup.bash

gnome-terminal  \
	--tab --title "pendulum_ctrl"	--command "bash -c \"
rosrun pendulum_ros pendulum_ros_node;
						exec bash\"" \
	--tab --title "pendulum_cfg"	--command "bash -c \"
rosrun rqt_reconfigure rqt_reconfigure;
						exec bash\"" &
