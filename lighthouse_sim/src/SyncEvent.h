#ifndef _SYNCEVENT_H_
#define _SYNCEVENT_H_

#include <gazebo/common/Event.hh>

using namespace gazebo;

event::EventT<void ()> sync_event;

#endif
