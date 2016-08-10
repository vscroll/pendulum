#ifndef _SYNCEVENT_H_
#define _SYNCEVENT_H_

#include <gazebo/common/Event.hh>

using namespace gazebo;

event::EventT<void ()> vf_sync;
event::EventT<void ()> hf_sync;

#endif
