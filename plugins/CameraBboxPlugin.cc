/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "gazebo/sensors/DepthCameraSensor.hh"
#include "CameraBboxPlugin.hh"
#include <ignition/math.hh>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CameraBboxPlugin);

/////////////////////////////////////////////////
CameraBboxPlugin::CameraBboxPlugin()
: SensorPlugin(), width(0), height(0), depth(0)
{
}

/////////////////////////////////////////////////
CameraBboxPlugin::~CameraBboxPlugin()
{
  this->parentSensor.reset();
  this->camera.reset();
}

/////////////////////////////////////////////////
void CameraBboxPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "CameraBboxPlugin requires a CameraSensor.\n";
    if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
      gzmsg << "It is a depth camera sensor\n";
  }

  this->camera = this->parentSensor->Camera();

  if (!this->parentSensor)
  {
    gzerr << "CameraBboxPlugin not attached to a camera sensor\n";
    return;
  }
  this->width = this->camera->ImageWidth();
  this->height = this->camera->ImageHeight();
  this->depth = this->camera->ImageDepth();
  this->format = this->camera->ImageFormat();

  // std::cout << pixelsI << '\n' << this->camera->WorldPose() << std::endl;
  this->connections.push_back(
     event::Events::ConnectPreRender(std::bind(&CameraBboxPlugin::Update, this)));
  this->newFrameConnection = this->camera->ConnectNewImageFrame(
      std::bind(&CameraBboxPlugin::OnNewFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void CameraBboxPlugin::OnNewFrame(const unsigned char * /*_image*/,
                              unsigned int /*_width*/,
                              unsigned int /*_height*/,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{
  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/camera/me.jpg");
    */

}

void CameraBboxPlugin::Update()
{

  ignition::math::Vector3d ptA, ptB, ptC, ptD, ptE, ptF, ptG , ptH;
  ptA.Set(4.775,0.4,0.2);
  ptB.Set(4.225,0.4,0.2);
  ptC.Set(4.225,-0.4,0.2);
  ptD.Set(4.775,-0.4,0.2);
  ptE.Set(4.775,0.4,2.1);
  ptF.Set(4.225,0.4,2.1);
  ptG.Set(4.225,-0.4,2.1);
  ptH.Set(4.775,-0.4,2.1);
  auto pixelsA = this->camera->Project(ptA);
  auto pixelsB = this->camera->Project(ptB);
  auto pixelsC = this->camera->Project(ptC);
  auto pixelsD = this->camera->Project(ptD);
  auto pixelsE = this->camera->Project(ptE);
  auto pixelsF = this->camera->Project(ptF);
  auto pixelsG = this->camera->Project(ptG);
  auto pixelsH = this->camera->Project(ptH);
  std::cout << "pixelsABCD" <<'\n'<< pixelsA << '\n' << pixelsB << "\n" << pixelsC << "\n" << pixelsD << "\n";
  std::cout << "pixelsEFGH" << '\n'<< pixelsE << '\n' << pixelsF << "\n" << pixelsG << "\n" << pixelsH << "\n";
  // ignition::math::Vector3d ptI;
  // ptI.Set(4.6,0,0);
  // auto pixelsI = this->camera->Project(ptI);
  // std::cout << pixelsI << '\n' << this->camera->WorldPose() << std::endl;

}
