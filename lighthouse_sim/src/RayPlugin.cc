/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Contact plugin
 * Author: Nate Koenig mod by John Hsu
 */

#include "gazebo/physics/physics.hh"
#include "RayPlugin.hh"
#include "SyncEvent.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RayPlugin)

/////////////////////////////////////////////////
RayPlugin::RayPlugin()
{
}

/////////////////////////////////////////////////
RayPlugin::~RayPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
}

void RayPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model = _model;
  world = model->GetWorld();

  fs = VFrame;
  last_frame_t = -1.0;

  if(_sdf->HasElement("linkName")) {
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    gzerr << "[RayPlugin] Please specify a linkName of the Ray\n";
  }

  link = model->GetLink(link_name);
  if (link == NULL) {
    gzthrow("[RayPlguin] Couldn't find specified link \"" << link_name << "\".");
  }

  updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&RayPlugin::OnUpdate, this, _1));
  printf("load Ray Plugin \n");
}

void RayPlugin::OnUpdate(const common::UpdateInfo& info)
{
  common::Time current_time = this->world->GetSimTime();
  double time = current_time.Double();

  if ((last_frame_t < 0) || (time - last_frame_t > 0.00833333)) {
    if (fs == VFrame) {
      math::Pose A(math::Vector3(0, 0, 0), math::Quaternion(1.57080, 0, 0.785398));
      //link->SetRelativePose(A);
      link->SetWorldPose(A);
      link->SetAngularVel(math::Vector3(133.287, -133.287, 0));
      fs = HFrame;
      printf("[RayPlugin] VFrame\n");
    } else {
      math::Pose A(math::Vector3(0, 0, 0), math::Quaternion(0, 0, 0));
      //link->SetRelativePose(A);
      link->SetWorldPose(A);
      link->SetAngularVel(math::Vector3(0, 0, 188.496));
      fs = VFrame;
      printf("[RayPlugin] HFrame\n");
    }
    //math::Vector3 v = link->GetWorldAngularVel();
    //math::Pose p = link->GetWorldCoGPose();
    //math::Vector3 e = p.rot.GetAsEuler();
    //printf("[RayPlugin] [ %f %f %f ] [ %f %f %f ]\n",  v[0], v[1], v[2], e[0], e[1], e[2]);

    sync_event();

    last_frame_t = time;
  }
}
