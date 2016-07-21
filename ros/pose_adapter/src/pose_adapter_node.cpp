#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <fmaros_msgs/PendulumPose.h>
#include <fmaros_msgs/VehiclePose.h>
#include "VehiclePendulumPose.pb.h"

#define PORT 14000

int main(int argc, char **argv)
{
  int sockfd,len;
  struct sockaddr_in addr;
  socklen_t  addr_len = sizeof(struct sockaddr_in);
  char buffer[4096];
  char buf[128];

  if((sockfd=socket(AF_INET,SOCK_DGRAM,0))<0){
	  perror ("socket");
	  exit(1);
  }

  bzero(&addr, sizeof(addr));
  addr.sin_family=AF_INET;
  addr.sin_port=htons(PORT);
  addr.sin_addr.s_addr=htonl(INADDR_ANY) ;
  if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr))<0){
	  perror("connect");
	  exit(1);
  }

  ros::init(argc, argv, "pose_adapter_node");

  ros::NodeHandle n;

  ros::Publisher pendulum_pub = n.advertise<fmaros_msgs::PendulumPose>("/FMA/Pendulum/Pose", 10);
  ros::Publisher vehicle_pub = n.advertise<fmaros_msgs::VehiclePose>("/FMA/Vehicle/Pose", 10);

  ros::Rate loop_rate(200);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();

    bzero(buffer,sizeof(buffer));
    len = recvfrom(sockfd,buffer,sizeof(buffer), 0 , (struct sockaddr *)&addr, &addr_len);

    if (len > 0) {
        vehiclependulum_msgs::msgs::VehiclePendulumPose pose_message_;

	pose_message_.ParseFromArray(buffer, len);
	int64_t time = pose_message_.time_usec();
	pendulum_msgs::msgs::PendulumPose pendulumpose = pose_message_.pendulum_pose();
	vehicle_msgs::msgs::VehiclePose vehiclepose =  pose_message_.vehicle_pose(0);

	fmaros_msgs::PendulumPose pendulum_pose;

	pendulum_pose.header.stamp.nsec = time;
	pendulum_pose.position.x = pendulumpose.x();
	pendulum_pose.position.y = pendulumpose.y();
	pendulum_pose.position.z = pendulumpose.z();
	pendulum_pose.velocity.x = pendulumpose.vx();
	pendulum_pose.velocity.y = pendulumpose.vy();
	pendulum_pose.velocity.z = pendulumpose.vz();
	pendulum_pose.vel_acc.x = pendulumpose.acc_x();
	pendulum_pose.vel_acc.y = pendulumpose.acc_y();
	pendulum_pose.vel_acc.z = pendulumpose.acc_z();

	pendulum_pub.publish(pendulum_pose);

	fmaros_msgs::VehiclePose vehicle_pose;

	vehicle_pose.position.x =vehiclepose.x();
	vehicle_pose.position.y =vehiclepose.y();
	vehicle_pose.position.z =vehiclepose.z();
	vehicle_pose.velocity.x =vehiclepose.vx();
	vehicle_pose.velocity.y =vehiclepose.vy();
	vehicle_pose.velocity.z =vehiclepose.vz();
	vehicle_pose.vel_acc.x =vehiclepose.acc_x();
	vehicle_pose.vel_acc.y =vehiclepose.acc_y();
	vehicle_pose.vel_acc.x =vehiclepose.acc_z();
	vehicle_pose.angle.x =vehiclepose.roll();
	vehicle_pose.angle.y =vehiclepose.pitch();
	vehicle_pose.angle.z =vehiclepose.yaw();
	vehicle_pose.ang_rate.x =vehiclepose.roll_rate();
	vehicle_pose.ang_rate.y =vehiclepose.pitch_rate();
	vehicle_pose.ang_rate.z =vehiclepose.yaw_rate();
	vehicle_pose.ang_rate_acc.x =vehiclepose.roll_acc();
	vehicle_pose.ang_rate_acc.y =vehiclepose.pitch_acc();
	vehicle_pose.ang_rate_acc.z =vehiclepose.yaw_acc();

	vehicle_pub.publish(vehicle_pose);
    }
  }

  return 0;
}
