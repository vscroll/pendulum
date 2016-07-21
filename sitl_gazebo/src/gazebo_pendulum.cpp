/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "gazebo_pendulum.h"

#define UDP_PORT 14000

namespace gazebo
{

PendulumPlugin::PendulumPlugin() {

}

PendulumPlugin::~PendulumPlugin() {
 event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void PendulumPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_pendulum] Please specify a robotNamespace.\n";

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);
 
  // Store the pointer to the model
  model_ = _model;

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_pendulum] Please specify a linkName of the pendulum.\n";

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_pendulum] Couldn't find specified link \"" << link_name_ << "\".");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&PendulumPlugin::OnUpdate, this, _1));

  vehiclepose_ = pose_message_.add_vehicle_pose();

  // try to setup udp socket for communcation with adaptor
  if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    printf("create socket failed\n");
    return;
  }

  reset_sub_ = node_handle_->Subscribe("reset", &PendulumPlugin::ResetCallback, this);

  // Set remote port
  memset((char *)&_srcaddr, 0, sizeof(_srcaddr));
  _srcaddr.sin_family = AF_INET;
  _srcaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  _srcaddr.sin_port = htons(UDP_PORT);
}

void PendulumPlugin::ResetCallback(ResetPtr& reset_msg)
{
  if (reset_msg->vehicle() == true) {
    // Reset Position
    model_->Reset();

    // Reset angular velocity
    model_->SetAngularVel(math::Vector3(0, 0, 0));

    // Reset linear velocity
    model_->SetLinearVel(math::Vector3(0, 0, 0));

    printf("[PendulumPlugin]:Reset Vehicle\n");
  }

  if (reset_msg->pendulum() == true) {
    // Reset angular velocity
    link_->SetAngularVel(math::Vector3(0, 0, 0));

    // Reset linear velocity
    link_->SetLinearVel(math::Vector3(0, 0, 0));

    math::Pose Pos(math::Vector3(0, 0, 0), math::Quaternion(0, 0, 0));
    // Reset position
    link_->SetRelativePose(Pos);

    printf("[PendulumPlugin]:Reset Pendulum\n");
  }
}

  // Called by the world update start event
void PendulumPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {
  // Get pendulum link status

  // Get the pose
  math::Pose link_pose = link_->GetWorldCoGPose();
  // Get the linear velocity
  math::Vector3 link_linear_vel = link_->GetWorldLinearVel();
  // Get the linear acceleration
  math::Vector3 link_linear_acc = link_->GetWorldLinearAccel();
  // Get the angular velocity
  //math::Vector3 link_angular_vel = link_->GetWorldAngularVel();
  // Get the angular acceleration
  //math::Vector3 link_angular_acc = link_->GetWorldAngularAccel();

  // Get base link status

  // Get the parent link
  physics::Link_V base_link = link_->GetParentJointsLinks();
  // Get the pose
  math::Pose model_pose = base_link.at(0)->GetWorldCoGPose();

  math::Vector3 euler = model_pose.rot.GetAsEuler();

  // Get the linear velocity
  math::Vector3 model_linear_vel = base_link.at(0)->GetWorldLinearVel();
  // Get the linear acceleration
  math::Vector3 model_linear_acc = base_link.at(0)->GetWorldLinearAccel();
  // Get the angular velocity
  math::Vector3 model_angular_vel = base_link.at(0)->GetWorldAngularVel();
  // Get the angular acceleration
  math::Vector3 model_angular_acc = base_link.at(0)->GetWorldAngularAccel();

//	math::Pose A(math::Vector3(0, 0, 0), math::Quaternion(0, 0.2, 0));
//	link_->SetRelativePose(A);
  // Test
  //base_link.at(0)->SetAngularVel(math::Vector3(0, 0, 5));

  common::Time current_time = model_->GetWorld()->GetSimTime();
  uint64_t usec = current_time.sec * 1000 + current_time.nsec / 1000000;

  // vehicle & pendulum pose
  pose_message_.set_time_usec(usec);

  // pendulum pose
  pendulum_msgs::msgs::PendulumPose* pendulumpose = new pendulum_msgs::msgs::PendulumPose();
  pendulumpose->set_time_usec(usec);
  pendulumpose->set_x(link_pose.pos[0]);
  pendulumpose->set_y(link_pose.pos[1]);
  pendulumpose->set_z(link_pose.pos[2]);
  pendulumpose->set_vx(link_linear_vel[0]);
  pendulumpose->set_vy(link_linear_vel[1]);
  pendulumpose->set_vz(link_linear_vel[2]);
  pendulumpose->set_acc_x(link_linear_acc[0]);
  pendulumpose->set_acc_y(link_linear_acc[1]);
  pendulumpose->set_acc_z(link_linear_acc[2]);

  // fill up with pendulum pose
  pose_message_.set_allocated_pendulum_pose(pendulumpose);

  // vehicle pose
  vehiclepose_->set_time_usec(usec);
  vehiclepose_->set_mav_sysid(99);
  vehiclepose_->set_x(model_pose.pos[0]);
  vehiclepose_->set_y(model_pose.pos[1]);
  vehiclepose_->set_z(model_pose.pos[2]);
  vehiclepose_->set_vx(model_linear_vel[0]);
  vehiclepose_->set_vy(model_linear_vel[1]);
  vehiclepose_->set_vz(model_linear_vel[2]);
  vehiclepose_->set_acc_x(model_linear_acc[0]);
  vehiclepose_->set_acc_y(model_linear_acc[1]);
  vehiclepose_->set_acc_z(model_linear_acc[2]);

  vehiclepose_->set_roll(euler[0]);
  vehiclepose_->set_pitch(euler[1]);
  vehiclepose_->set_yaw(euler[2]);
  vehiclepose_->set_roll_rate(model_angular_vel[0]);
  vehiclepose_->set_pitch_rate(model_angular_vel[1]);
  vehiclepose_->set_yaw_rate(model_angular_vel[2]);
  vehiclepose_->set_roll_acc(model_angular_acc[0]);
  vehiclepose_->set_pitch_acc(model_angular_acc[1]);
  vehiclepose_->set_yaw_acc(model_angular_acc[2]);

  int size = pose_message_.ByteSize();
  char buffer[size];
  pose_message_.SerializeToArray(buffer, size);

  int len = 0;

  len = sendto(_fd, buffer, size, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));

  if (len <= 0) {
    printf("Failed sending mavlink message");
  }
}
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PendulumPlugin);
}
