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
  /// \brief A Ray Sensor Plugin
  class RayPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: RayPlugin();

    /// \brief Destructor
    public: virtual ~RayPlugin();

    /// \brief Load the plugin
    /// \param take in SDF root element
    private:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    /// \brief Pointer to world
    std::string link_name;
    physics::WorldPtr world;
    physics::ModelPtr model;
    physics::LinkPtr link;

    enum FrameStatus
    {
      // Vertecal Frame
      VFrame,
      // Horizontal Frame
      HFrame
    };

    FrameStatus fs;
    double last_frame_t;

    event::ConnectionPtr updateConnection;

    void OnUpdate(const common::UpdateInfo& info);

    pthread_t thread;

    void* thread_run(void* arg);
  };
}
#endif
