#ifndef GAZEBO_PLUGINS_GETPOSE_PLUGIN_HH_
#define GAZEBO_PLUGINS_GETPOSE_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

namespace gazebo
{
  class GetPosePlugin : public ModelPlugin
  {

  public: GetPosePlugin();
  public: virtual ~GetPosePlugin();
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
  public: void Update();

  //Store pointer to the model
  private: physics::ModelPtr model;
  private: event::ConnectionPtr updateConnection;

  /// Ros stuff:
  private: ros::NodeHandle nh;
  private: ros::Publisher pose_pub;
  private:geometry_msgs::PoseStamped posedata;

};
}
#endif
