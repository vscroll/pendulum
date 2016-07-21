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

#ifndef _GAZEBO_PENDULUM_H_
#define _GAZEBO_PENDULUM_H_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include "VehiclePendulumPose.pb.h"
#include "Reset.pb.h"

#include <sys/socket.h>
#include <netinet/in.h>

namespace gazebo {

  typedef const boost::shared_ptr<const reset_msgs::msgs::Reset> ResetPtr;

  class PendulumPlugin : public ModelPlugin {
    public:
    PendulumPlugin();
    ~PendulumPlugin();

    protected:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo&);

    private:
    // socket
    int _fd;
    struct sockaddr_in _srcaddr;

    std::string namespace_;
    std::string link_name_;
    // Pointer to the model
    physics::ModelPtr model_;
    // Pointer to the link
    physics::LinkPtr link_;
    // Point to the update event connection
    event::ConnectionPtr updateConnection_;

    transport::NodePtr node_handle_;

    transport::SubscriberPtr reset_sub_;

    // vehicle pendulum pose message
    vehiclependulum_msgs::msgs::VehiclePendulumPose pose_message_;
    // vehicle pose message
    vehicle_msgs::msgs::VehiclePose* vehiclepose_;
    // pendulum pose message not declare here

    void ResetCallback(ResetPtr& reset_msg);
  };
}
#endif
