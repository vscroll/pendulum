#include "TargetScan.h"
#include "ScanEvent.h"
#include "SyncEvent.h"

namespace gazebo
{

TargetScan::TargetScan() {

}

TargetScan::~TargetScan() {
  sync_event.Disconnect(sync);
}

void TargetScan::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {
  world = _parent;

  sync = sync_event.Connect(boost::bind(&TargetScan::SyncUpdate, this));

  printf("load TargetScan Plugin\n");
  pthread_create(&thread, NULL, &TargetScan::thread_run, this);
}

void TargetScan::SyncUpdate() {
  common::Time current_time = world->GetSimTime();
  double time = current_time.Double();

  //printf("[TargetScan] Sync time %lf\n", time);
}

void TargetScan::Scan() {

  while (1) {
    common::Time current_time = this->world->GetSimTime();
    double time = current_time.Double();

    scan_event();
    //printf("[TargetScan] Scan time %lf\n", time);

    usleep(100);
  }
}

void* TargetScan::thread_run(void *arg) {
  ((TargetScan *)arg)->Scan();
  return NULL;
}

  GZ_REGISTER_WORLD_PLUGIN(TargetScan);
}
