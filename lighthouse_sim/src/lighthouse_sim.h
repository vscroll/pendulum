#ifndef _LIGHTHOUSE_SIM_H_
#define _LIGHTHOUSE_SIM_H_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class LightHouse : public ModelPlugin {
    public:
    LightHouse();
    ~LightHouse();

    protected:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private:

  };
}
#endif
