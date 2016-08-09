#include "TargetScan.h"
#include "ScanEvent.h"
#include "SyncEvent.h"
#include "Common.h"

namespace gazebo
{

TargetScan::TargetScan() {

}

TargetScan::~TargetScan() {
  sync_event.Disconnect(sync);
}

void TargetScan::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
  world = _parent;
  lenAB = 0.0;
  lenBC = 0.0;
  lenAC = 0.0;

  sync_t = 0.0;

  if(_sdf->HasElement("rayModelName")) {
    rayModelName = _sdf->GetElement("rayModelName")->Get<std::string>();
  } else {
    gzerr << "[RayPlugin] Please specify a modelName of the Ray\n";
  }
  rayModel = world->GetModel(rayModelName);

  rayLink = rayModel->GetLink("top");
  if (rayLink == NULL)
    gzthrow("[RayPlugin] Couldn't find specified link.");

  if(_sdf->HasElement("modelNameA")) {
    modelNameA = _sdf->GetElement("modelNameA")->Get<std::string>();
  } else {
    gzerr << "[RayPlugin] Please specify a modelName of the target\n";
  }
  modelA = world->GetModel(modelNameA);

  linkA = modelA->GetLink("link");
  if (linkA == NULL)
    gzthrow("[RayPlugin] Couldn't find specified link.");

  if(_sdf->HasElement("modelNameB")) {
    modelNameB = _sdf->GetElement("modelNameB")->Get<std::string>();
  } else {
    gzerr << "[RayPlugin] Please specify a modelName of the target\n";
  }
  modelB = world->GetModel(modelNameB);

  linkB = modelB->GetLink("link");
  if (linkB == NULL)
    gzthrow("[RayPlugin] Couldn't find specified link.");

  if(_sdf->HasElement("modelNameC")) {
    modelNameC = _sdf->GetElement("modelNameC")->Get<std::string>();
  } else {
    gzerr << "[RayPlugin] Please specify a modelName of the target\n";
  }
  modelC = world->GetModel(modelNameC);

  linkC = modelC->GetLink("link");
  if (linkC == NULL)
    gzthrow("[RayPlugin] Couldn't find specified link.");

  sync = sync_event.Connect(boost::bind(&TargetScan::SyncUpdate, this));

  pthread_create(&thread, NULL, &TargetScan::thread_run, this);
  printf("load TargetScan Plugin\n");
}

void TargetScan::SyncUpdate() {
  common::Time current_time = world->GetSimTime();
  double time = current_time.Double();
  sync_t = time;
  printf("[TargetScan] Sync time %lf\n", time);
}

void TargetScan::Scan() {
  while (1) {
    common::Time current_time = this->world->GetSimTime();
    double time = current_time.Double();

    math::Pose pA = linkA->GetWorldCoGPose();
    math::Pose pB = linkB->GetWorldCoGPose();
    math::Pose pC = linkC->GetWorldCoGPose();

    lenAB = math::Vector3(pA.pos[0] - pB.pos[0], pA.pos[1] - pB.pos[1], pA.pos[2] - pB.pos[2]).GetLength();
    lenBC = math::Vector3(pB.pos[0] - pC.pos[0], pB.pos[1] - pC.pos[1], pB.pos[2] - pC.pos[2]).GetLength();
    lenAC = math::Vector3(pA.pos[0] - pC.pos[0], pA.pos[1] - pC.pos[1], pA.pos[2] - pC.pos[2]).GetLength();
    //printf("[TargetScan] AB %lf BC %lf AC %lf\n", lenAB, lenBC, lenAC);
    math::Pose pR = rayLink->GetWorldCoGPose();

    double lAR = math::Vector2d(pA.pos[0], pA.pos[1]).Distance(math::Vector2d(pR.pos[0], pR.pos[1]));
    double angA = atan2(pA.pos[2] - pR.pos[2], lAR);
    //printf("angA%f,pA[%f %f %f] pR[%f %f %f]\n", angA, pA.pos[0], pA.pos[1], pA.pos[2], pR.pos[0], pR.pos[1], pR.pos[2]);

    double lBR = math::Vector2d(pB.pos[0], pB.pos[1]).Distance(math::Vector2d(pR.pos[0], pR.pos[1]));
    double angB = atan2(pB.pos[2] - pR.pos[2], lBR);
    //printf("angB%f,pB[%f %f %f] pR[%f %f %f]\n", angB, pB.pos[0], pB.pos[1], pB.pos[2], pR.pos[0], pR.pos[1], pR.pos[2]);

    double lCR = math::Vector2d(pC.pos[0], pC.pos[1]).Distance(math::Vector2d(pR.pos[0], pR.pos[1]));
    double angC = atan2(pC.pos[2] - pR.pos[2], lCR);
    //printf("angC%f,pC[%f %f %f] pR[%f %f %f]\n", angC, pC.pos[0], pC.pos[1], pC.pos[2], pR.pos[0], pR.pos[1], pR.pos[2]);

    math::Vector3 angR = pR.rot.GetAsEuler();
    //printf("angR[%f %f %f]\n", angR[0], -angR[1], angR[2]);

    if(fabs(angA - fabs(angR[1])) < 0.00005) {
      double angAA = (time - sync_t) * SCAN_ANGULAR_VEL;
      printf("angA' angA [%f %f]\n", -angR[1], angAA);
      //publish
    }

    if(fabs(angB - fabs(angR[1])) < 0.00005) {
      double angBB = (time - sync_t) * SCAN_ANGULAR_VEL;
      printf("angB' angB [%f %f]\n", -angR[1], angBB);
      //publish
    }

    if(fabs(angC - fabs(angR[1])) < 0.00005) {
      double angCC = (time - sync_t) * SCAN_ANGULAR_VEL;
      printf("angC' angC [%f %f]\n", -angR[1], angCC);
      //publish
    }

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
