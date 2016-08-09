#ifndef _TARGETSCAN_H_
#define _TARGETSCAN_H_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class TargetScan : public WorldPlugin {
    public:
    TargetScan();
    ~TargetScan();

    void Scan();

    protected:
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    private:
    std::string modelNameA;
    std::string modelNameB;
    std::string modelNameC;
    physics::ModelPtr modelA;
    physics::ModelPtr modelB;
    physics::ModelPtr modelC;
    physics::LinkPtr linkA;
    physics::LinkPtr linkB;
    physics::LinkPtr linkC;

    std::string rayModelName;
    physics::ModelPtr rayModel;
    physics::LinkPtr rayLink;

    double lenAB;
    double lenBC;
    double lenAC;

    event::ConnectionPtr sync;
    physics::WorldPtr world;
    pthread_t thread;
    double sync_t;
    void SyncUpdate();
    static void* thread_run(void *arg);
  };
}
#endif
