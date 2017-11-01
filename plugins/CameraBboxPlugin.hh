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
#ifndef GAZEBO_PLUGINS_CAMERABBOXPLUGIN_HH_
#define GAZEBO_PLUGINS_CAMERABBOXPLUGIN_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/common/Events.hh"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "vision_msgs/Detection2D.h"

#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo
{

  class CameraBboxPlugin : public CameraPlugin, GazeboRosCameraUtils
  {
    public: CameraBboxPlugin();

    /// \brief Destructor
    public:  ~CameraBboxPlugin();
    public:  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    public: virtual void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    // protected: unsigned int width, height, depth;
    // protected: std::string format;
    //
    // protected: sensors::CameraSensorPtr parentSensor;
    // protected: rendering::CameraPtr camera;
    public: void Update();
    public: void Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    private: event::ConnectionPtr newFrameConnection;
    protected: std::vector<event::ConnectionPtr> connections;

    public: std::vector<double> d;
    public: std::mutex mutex;
    public: bool dirty = false;
    public: ros::NodeHandle nh;
    public: ros::Publisher pub;
    public: ros::Subscriber sub;
    public: std::vector<int> datax;
    public: std::vector<int> datay;
    public: vision_msgs::Detection2D box;

  };
}
#endif
