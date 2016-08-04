#include "TargetScan.h"
#include "ScanEvent.h"

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
    usleep(1000000);
    scan_event();
  }

  return NULL;
}

  GZ_REGISTER_WORLD_PLUGIN(TargetScan);
}
