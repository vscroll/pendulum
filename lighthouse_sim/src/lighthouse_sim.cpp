#include "lighthouse_sim.h"

namespace gazebo
{

LightHouse::LightHouse() {

}

LightHouse::~LightHouse() {
}

void LightHouse::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  printf("load LightHouse Plugin\n");
}

  GZ_REGISTER_MODEL_PLUGIN(LightHouse);
}
