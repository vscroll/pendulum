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

void RayPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;
  this->world = this->model->GetWorld();

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&RayPlugin::OnUpdate, this, _1));
}

void RayPlugin::OnUpdate(const common::UpdateInfo& info)
{
  common::Time current_time = this->world->GetSimTime();
  double time = current_time.Double();
  sync_event();
  printf("[RayPlugin] Sync time %lf\n", time);
}
