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
    void CollisionUpdate(const common::UpdateInfo&);

    private:
    std::string namespace_;
    std::string link_name_;
    // Pointer to the model
    physics::ModelPtr model_;
    // Pointer to the link
    physics::LinkPtr link_;
    // Point to the collision detection event connection
    event::ConnectionPtr physicsUpdate_;
  };
}
#endif
