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

GZ_REGISTER_SENSOR_PLUGIN(RayPlugin)

RayPlugin::RayPlugin()
{
}

RayPlugin::~RayPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
}

void RayPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  fs = VFrame;
  last_frame_t = -1.0;

  // Get then name of the parent sensor
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parentSensor)
    gzthrow("RayPlugin requires a Ray Sensor as its parent");

  this->world = physics::get_world(this->parentSensor->GetWorldName());

  if(_sdf->HasElement("modelName")) {
    model_name = _sdf->GetElement("modelName")->Get<std::string>();
  } else {
    gzerr << "[RayPlugin] Please specify a modelName of the Ray\n";
  }

  this->model = this->world->GetModel(model_name);

  if(_sdf->HasElement("linkName")) {
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    gzerr << "[RayPlugin] Please specify a linkName of the Ray\n";
  }

  this->link = this->model->GetLink(link_name);
  if (this->link == NULL) {
    gzthrow("[RayPlguin] Couldn't find specified link \"" << link_name << "\".");
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&RayPlugin::OnUpdate, this, _1));

  printf("RayPlugin loaded\n");
}

void RayPlugin::OnUpdate(const common::UpdateInfo& info)
{
  common::Time current_time = this->world->GetSimTime();
  double time = current_time.Double();

  if ((last_frame_t < 0) || (time - last_frame_t > 0.00833333)) {
    if (fs == VFrame) {
      math::Pose A(math::Vector3(0, 0, 0), math::Quaternion(1.57080, 0, 0.785398));
      link->SetWorldPose(A);
      //SCAN_ANGULAR_VEL^2 = 2 * 133.287^2
      link->SetAngularVel(math::Vector3(133.287, -133.287, 0));
      fs = HFrame;
      vf_sync();
      printf("[RayPlugin] VFrame\n");
    } else {
      math::Pose A(math::Vector3(0, 0, 0), math::Quaternion(0, 0, 0));
      link->SetWorldPose(A);
      link->SetAngularVel(math::Vector3(0, 0, SCAN_ANGULAR_VEL));
      fs = VFrame;
      hf_sync();
      printf("[RayPlugin] HFrame\n");
    }

    last_frame_t = time;
  }
}
