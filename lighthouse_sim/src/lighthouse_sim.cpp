#include "lighthouse_sim.h"
#include "ScanEvent.h"

namespace gazebo
{

LightHouse::LightHouse() {

}

LightHouse::~LightHouse() {
  scan_event.Disconnect(Scan);
}

void LightHouse::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
  Scan = scan_event.Connect(boost::bind(&LightHouse::Update, this));
  printf("load LightHouse Plugin\n");
}

void LightHouse::Update() {
  //printf("LightHouse Scan\n");
}

  GZ_REGISTER_WORLD_PLUGIN(LightHouse);
}
