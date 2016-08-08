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
    event::ConnectionPtr sync;
    physics::WorldPtr world;
    pthread_t thread;
    void SyncUpdate();
    static void* thread_run(void *arg);
  };
}
#endif
