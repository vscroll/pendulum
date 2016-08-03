#include "lighthouse_sim.h"

namespace gazebo
{

LightHouse::LightHouse() {

}

LightHouse::~LightHouse() {
  event::Events::DisconnectBeforePhysicsUpdate(physicsUpdate_);
}

void LightHouse::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  physicsUpdate_ = event::Events::ConnectBeforePhysicsUpdate(
    boost::bind(&LightHouse::CollisionUpdate, this, _1));
  printf("load LightHouse Plugin\n");
}

void LightHouse::CollisionUpdate(const common::UpdateInfo & /*info*/) {
  printf("collision detection\n");

}

  GZ_REGISTER_MODEL_PLUGIN(LightHouse);
}
