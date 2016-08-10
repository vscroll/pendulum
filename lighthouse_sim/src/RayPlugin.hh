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
 * Desc: Ray Plugin
 * Author: Nate Koenig mod by John Hsu
 */

#ifndef _GAZEBO_RAY_PLUGIN_HH_
#define _GAZEBO_RAY_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE RayPlugin : public SensorPlugin
  {
    public:
    RayPlugin();
    ~RayPlugin();

    private:
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo& info);

    enum FrameStatus
    {
      // Vertecal Frame
      VFrame,
      // Horizontal Frame
      HFrame
    };

    FrameStatus fs;
    double last_frame_t;

    std::string model_name;
    std::string link_name;

    physics::WorldPtr world;
    physics::ModelPtr model;
    physics::LinkPtr link;

    sensors::RaySensorPtr parentSensor;
    event::ConnectionPtr updateConnection;
  };
}
#endif
