/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "gazebo_reset_plugin.h"
#include "Reset.pb.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GUIResetWidget)

/////////////////////////////////////////////////
GUIResetWidget::GUIResetWidget()
  : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  QPushButton *button = new QPushButton(tr("Reset"));
  connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));

  // Create a vehicle reset check box
  vehicle_checkbox = new QCheckBox(tr("Vehicle"));

  // Create a pendulum reset check box
  pendulum_checkbox = new QCheckBox(tr("Pendulum"));

  // Add the vehicle box to the frame's layout
  frameLayout->addWidget(vehicle_checkbox);

  // Add the pendulum box to the frame's layout
  frameLayout->addWidget(pendulum_checkbox);

  // Add the button to the frame's layout
  frameLayout->addWidget(button);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(120, 80);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->resetPub = this->node->Advertise<reset_msgs::msgs::Reset>("reset");
}

/////////////////////////////////////////////////
GUIResetWidget::~GUIResetWidget()
{
}

/////////////////////////////////////////////////
void GUIResetWidget::OnButton()
{
  reset_msgs::msgs::Reset reset;

  if (vehicle_checkbox->isChecked()) {
    reset.set_vehicle(true);
  } else {
    reset.set_vehicle(false);
  }

  if (pendulum_checkbox->isChecked()) {
    reset.set_pendulum(true);
  } else {
    reset.set_pendulum(false);
  }

  this->resetPub->Publish(reset);
}
