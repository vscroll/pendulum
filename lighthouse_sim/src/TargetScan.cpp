#include "TargetScan.h"


namespace gazebo
{

TargetScan::TargetScan() {

}

TargetScan::~TargetScan() {
}

void TargetScan::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {
  world = _parent;

  printf("load TargetScan Plugin\n");
  pthread_create(&thread, NULL, &TargetScan::Scan, this);
}

void* TargetScan::Scan(void *arg) {
  while (1) {
    usleep(100);
  }

  return NULL;
}

  GZ_REGISTER_WORLD_PLUGIN(TargetScan);
}
