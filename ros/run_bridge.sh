#! /bin/bash

#run in dir catkin_ws

STACK_PATH=./
source /opt/ros/indigo/setup.bash
source ${STACK_PATH}/devel/setup.bash

gnome-terminal  \
	--tab --title "roscore"	--command "bash -c \"
roscore;
						exec bash\"" \
	--tab --title "mavros"	--command "bash -c \"
rosrun mavros mavros_node _fcu_url:=udp://localhost:14540@localhost:14557 _gcs_url:=udp://@localhost:14550;
						exec bash\"" \
	--tab --title "pose_adapter"	--command "bash -c \"
rosrun pose_adapter pose_adapter_node;
						exec bash\"" &
