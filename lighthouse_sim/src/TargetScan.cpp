#include "TargetScan.h"
#include "SyncEvent.h"

namespace gazebo
{

TargetScan::TargetScan() {

}

TargetScan::~TargetScan() {
  vf_sync.Disconnect(vf_syncCon);
  hf_sync.Disconnect(hf_syncCon);
}

void TargetScan::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
  world = _parent;

  lenAB = 0.0;
  lenBC = 0.0;
  lenAC = 0.0;

  sync_t = 0.0;
  memset(angDectedtedList, 0, sizeof(angDectedtedList));
  memset(angList, 0, sizeof(angList));

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

  vf_syncCon = vf_sync.Connect(boost::bind(&TargetScan::VF_SyncUpdate, this));
  hf_syncCon = hf_sync.Connect(boost::bind(&TargetScan::HF_SyncUpdate, this));

  pthread_create(&thread, NULL, &TargetScan::thread_run, this);
  printf("load TargetScan Plugin\n");
}

void TargetScan::updateSyncTime() {
  common::Time current_time = world->GetSimTime();
  double time = current_time.Double();
  sync_t = time;
  memset(angDectedtedList, 0, sizeof(angDectedtedList));
  //memset(angList, 0, sizeof(angList));
  //printf("[TargetScan]Sync time %lf\n", time);
}

void TargetScan::VF_SyncUpdate() {
  updateSyncTime();
  fs = VFrame;
}

void TargetScan::HF_SyncUpdate() {
  updateSyncTime();
  fs = HFrame;
}

void TargetScan::coorUpdate() {
  int i = 0;
  int j = 0;

  for(i; i < 2; i++) {
    for (j; j < 3; j++) {
      if(!angDectedtedList[i][j]) {
        //detecte incompletly
        return;
      }
    }
  }

  //calculate coordinate

  /*
     |z
     |  /P
     | /
     |/_|b_______y
    /-\ |
   / a \
  /x    \

   *------->  ---------------------------------->
   *(x,y,x) = (cos(b)*cos(a), cos(b)sin(a),sin(b))
  */

  //calculate angular between two lines
  double x0 = cos(angList[0][0])*cos(angList[1][0]);
  double y0 = cos(angList[0][0])*sin(angList[1][0]);
  double z0 = sin(angList[0][0]);

  double x1 = cos(angList[0][1])*cos(angList[1][1]);
  double y1 = cos(angList[0][1])*sin(angList[1][1]);
  double z1 = sin(angList[0][1]);

  double len0 = x0*x0+y0*y0+z0*z0;
  double len1 = x1*x1+y1*y1+z1*z1;
  double x = x0*x1+y0*y1+z0*z1;

  double cosa = x / (sqrtf(len0)*sqrtf(len1));
  double ang1 = acos(cosa) * 57.3;
  math::Pose pA = linkA->GetWorldCoGPose();
  math::Pose pB = linkB->GetWorldCoGPose();
  math::Pose pR = rayLink->GetWorldCoGPose();

  math::Vector3 OA = pA.pos - pR.pos;
  math::Vector3 OB = pB.pos - pR.pos;
  double len02 = OA[0]*OA[0]+OA[1]*OA[1]+OA[2]*OA[2];
  double len12 = OB[0]*OB[0]+OB[1]*OB[1]+OB[2]*OB[2];
  double x2 = OA[0]*OB[0]+OA[1]*OB[1]+OA[2]*OB[2];
  double cosa2 = x2 / (sqrtf(len02)*sqrtf(len12));
  double ang2 = acos(cosa2) * 57.3;

  printf("OA[%f %f %f]\nOB[%f %f %f]\nANG1 %f ANG2 %f\n", OA[0], OA[1], OA[2], OB[0], OB[1],OB[2], ang1, ang2);

  //publish coordinate
}

void TargetScan::Scan() {
  while (1) {
    common::Time current_time = this->world->GetSimTime();
    double time = current_time.Double();

    math::Pose pA = linkA->GetWorldCoGPose();
    math::Pose pB = linkB->GetWorldCoGPose();
    math::Pose pC = linkC->GetWorldCoGPose();
    math::Pose pR = rayLink->GetWorldCoGPose();
    math::Vector3 angR = pR.rot.GetAsEuler();
    //printf("angR[%f %f %f]\n", angR[0], -angR[1], angR[2]);

    if (fs == VFrame) {
      double lAR = math::Vector2d(pA.pos[0], pA.pos[1]).Distance(math::Vector2d(pR.pos[0], pR.pos[1]));
      double angA = atan2(pA.pos[2] - pR.pos[2], lAR);
      //printf("angA%f,pA[%f %f %f] pR[%f %f %f]\n", angA, pA.pos[0], pA.pos[1], pA.pos[2], pR.pos[0], pR.pos[1], pR.pos[2]);

      double lBR = math::Vector2d(pB.pos[0], pB.pos[1]).Distance(math::Vector2d(pR.pos[0], pR.pos[1]));
      double angB = atan2(pB.pos[2] - pR.pos[2], lBR);
      //printf("angB%f,pB[%f %f %f] pR[%f %f %f]\n", angB, pB.pos[0], pB.pos[1], pB.pos[2], pR.pos[0], pR.pos[1], pR.pos[2]);

      double lCR = math::Vector2d(pC.pos[0], pC.pos[1]).Distance(math::Vector2d(pR.pos[0], pR.pos[1]));
      double angC = atan2(pC.pos[2] - pR.pos[2], lCR);
      //printf("angC%f,pC[%f %f %f] pR[%f %f %f]\n", angC, pC.pos[0], pC.pos[1], pC.pos[2], pR.pos[0], pR.pos[1], pR.pos[2]);

      if(!angDectedtedList[0][0]  && fabs(angA - fabs(angR[1])) < 0.00005) {
        angList[0][0] = (time - sync_t) * SCAN_ANGULAR_VEL;
        printf("angA' angA [%f %f]\n", -angR[1], angList[0][0]);
        angDectedtedList[0][0] = true;
      }

      if(!angDectedtedList[0][1] && fabs(angB - fabs(angR[1])) < 0.00005) {
        angList[0][1] = (time - sync_t) * SCAN_ANGULAR_VEL;
        printf("angB' angB [%f %f]\n", -angR[1], angList[0][1]);
        angDectedtedList[0][1] = true;
      }

      if(!angDectedtedList[0][2] && fabs(angC - fabs(angR[1])) < 0.00005) {
        angList[0][2]= (time - sync_t) * SCAN_ANGULAR_VEL;
        printf("angC' angC [%f %f]\n", -angR[1], angList[0][2]);
        angDectedtedList[0][2] = true;
      }

    } else if (fs == HFrame){
      double angA = atan2(pA.pos[1] - pR.pos[1], pA.pos[0] - pR.pos[0]);
      //printf("angA%f,pA[%f %f] pR[%f %f]\n", angA, pA.pos[0], pA.pos[1], pR.pos[0], pR.pos[1]);

      double angB = atan2(pB.pos[1] - pR.pos[1], pB.pos[0] - pR.pos[0]);
      //printf("angB%f,pB[%f %f] pR[%f %f]\n", angB, pB.pos[0], pB.pos[1], pR.pos[0], pR.pos[1]);

      double angC = atan2(pC.pos[1] - pR.pos[1], pC.pos[0] - pR.pos[0]);
      //printf("angC%f,pC[%f %f] pR[%f %f]\n", angC, pC.pos[0], pC.pos[1], pR.pos[0], pR.pos[1]);

      if(!angDectedtedList[1][0] && fabs(angA - fabs(angR[2])) < 0.00005) {
        angList[1][0]= (time - sync_t) * SCAN_ANGULAR_VEL;
        printf("angA' angA [%f %f]\n", angR[2], angList[1][0]);
        angDectedtedList[1][0] = true;
      }

      if(!angDectedtedList[1][1] && fabs(angB - fabs(angR[2])) < 0.00005) {
        angList[1][1]= (time - sync_t) * SCAN_ANGULAR_VEL;
        printf("angB' angB [%f %f]\n", angR[2], angList[1][1]);
        angDectedtedList[1][1] = true;
      }

      if(!angDectedtedList[1][2] && fabs(angC - fabs(angR[2])) < 0.00005) {
        angList[1][2] = (time - sync_t) * SCAN_ANGULAR_VEL;
        printf("angC' angC [%f %f]\n", angR[2], angList[1][2]);
        angDectedtedList[1][2] = true;
      }
   }

    coorUpdate();

    usleep(100);
  }
}

void* TargetScan::thread_run(void *arg) {
  ((TargetScan *)arg)->Scan();
  return NULL;
}

  GZ_REGISTER_WORLD_PLUGIN(TargetScan);
}
