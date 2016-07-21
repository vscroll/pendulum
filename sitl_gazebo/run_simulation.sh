#! /bin/bash

gnome-terminal  \
	--tab --title "gazebo"	--command "bash -c \"
make posix_sitl_default gazebo;
						exec bash\"" &
