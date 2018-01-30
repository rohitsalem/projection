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
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/Sensor.hh>
#include <ignition/math.hh>
#include <string>
#include <algorithm>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CameraBboxPlugin);

/////////////////////////////////////////////////
CameraBboxPlugin::CameraBboxPlugin()
// : SensorPlugin(), width(0), height(0), depth(0)
{
  sub = nh.subscribe("objectBoxWorldCoordinates",1 , &CameraBboxPlugin::Callback , this);
  pub = nh.advertise<vision_msgs::Detection2D>("bounding_box",1);
}


/////////////////////////////////////////////////
CameraBboxPlugin::~CameraBboxPlugin()
{
  ROS_DEBUG_STREAM_NAMED("camera", "Unloaded");
  // this->parentSensor.reset();
  // this->camera.reset();
}

/////////////////////////////////////////////////
void CameraBboxPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("camera", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
}
  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;

  GazeboRosCameraUtils::Load(_parent, _sdf);
  // this->connections.push_back(event::Events::ConnectPreRender(std::bind(&CameraBboxPlugin::Update, this)));

}
/////////////////////////////////////////////////
void CameraBboxPlugin::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
# if GAZEBO_MAJOR_VERSION >= 7
  common::Time sensor_update_time = this->parentSensor_->LastMeasurementTime();
# else
  common::Time sensor_update_time = this->parentSensor_->GetLastMeasurementTime();
# endif

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
    {
      // OnNewFrame is triggered at the gazebo sensor <update_rate>
      // while there is also a plugin <updateRate> that can throttle the
      // rate down further (but then why not reduce the sensor rate?
      // what is the use case?).
      // Setting the <updateRate> to zero will make this plugin
      // update at the gazebo sensor <update_rate>, update_period_ will be
      // zero and the conditional always will be true.
      if (sensor_update_time - this->last_update_time_ >= this->update_period_)
      {
        this->PutCameraData(_image, sensor_update_time);
        this->PublishCameraInfo(sensor_update_time);
        this->last_update_time_ = sensor_update_time;
      }
    }
  }
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->dirty)
  {
    ignition::math::Vector3d ptA, ptB, ptC, ptD, ptE, ptF, ptG , ptH;
    ptA.Set(d[0],d[1],d[2]);
    ptB.Set(d[3],d[4],d[5]);
    ptC.Set(d[6],d[7],d[8]);
    ptD.Set(d[9],d[10],d[11]);
    ptE.Set(d[12],d[13],d[14]);
    ptF.Set(d[15],d[16],d[17]);
    ptG.Set(d[18],d[19],d[20]);
    ptH.Set(d[21],d[22],d[23]);
    auto pixelsA = this->camera->Project(ptA);
    auto pixelsB = this->camera->Project(ptB);
    auto pixelsC = this->camera->Project(ptC);
    auto pixelsD = this->camera->Project(ptD);
    auto pixelsE = this->camera->Project(ptE);
    auto pixelsF = this->camera->Project(ptF);
    auto pixelsG = this->camera->Project(ptG);
    auto pixelsH = this->camera->Project(ptH);
    datax.clear();  // clear the contents before publishing a new message
    datay.clear();
    datax.push_back(pixelsA[0]);
    datay.push_back(pixelsA[1]);
    datax.push_back(pixelsB[0]);
    datay.push_back(pixelsB[1]);
    datax.push_back(pixelsC[0]);
    datay.push_back(pixelsC[1]);
    datax.push_back(pixelsD[0]);
    datay.push_back(pixelsD[1]);
    datax.push_back(pixelsE[0]);
    datay.push_back(pixelsE[1]);
    datax.push_back(pixelsF[0]);
    datay.push_back(pixelsF[1]);
    datax.push_back(pixelsG[0]);
    datay.push_back(pixelsG[1]);
    datax.push_back(pixelsH[0]);
    datay.push_back(pixelsH[1]);

    auto minx = *std::min_element(datax.begin(),datax.end());  //min x
    auto maxx = *std::max_element(datax.begin(),datax.end());  //max x
    auto miny = *std::min_element(datay.begin(),datay.end());  //min y
    auto maxy = *std::max_element(datay.begin(),datay.end());  //max y

    box.header.stamp.sec = sec ;
    box.header.stamp.nsec = nsec;
    box.bbox.center.x = int((minx + maxx)/2);
    box.bbox.center.y = int((miny + maxy)/2);
    box.bbox.size_x = maxx - minx;
    box.bbox.size_y = maxy - miny;

    //double secs = ros::Time::now().toSec();
    // double nsecs = ros::Time::now().toNsec();
    // std::cout << "Difference Secs: " <<  secs-sensor_update_time.sec << '\n';
    // std::cout << "Difference nSecs: " <<  nsecs-sensor_update_time.nsec << '\n';
    // std::cout << "Difference bbox secs" << sec-sensor_update_time.sec << '\n';
    // std::cout << "Difference bbox nsecs" << nsec-sensor_update_time.nsec << '\n';

    // if (sec == sensor_update_time.sec && nsec == sensor_update_time.nsec)
    // {
      // std::cout << "publishing box" << '\n';
    this->pub.publish(box);
    // }
    this->dirty = false;
  }
}

void CameraBboxPlugin::Callback(const projection::Float64MultiArrayStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  d = msg->array.data;
  sec= msg->header.stamp.sec;
  nsec= msg->header.stamp.nsec;

  this->dirty = true; // FLag to enable mutex

}
