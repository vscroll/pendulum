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
  pthread_create(&thread, NULL, &TargetScan::thread_run, this);
}

void TargetScan::Scan() {

  while (1) {
    common::Time current_time = this->world->GetSimTime();
    double time = current_time.Double();
    printf("RayPlugin::OnNewLaserScans time %lf\n", time);

    usleep(1000000);
    scan_event();
  }
}

void* TargetScan::thread_run(void *arg) {
  ((TargetScan *)arg)->Scan();
  return NULL;
}

  GZ_REGISTER_WORLD_PLUGIN(TargetScan);
}
