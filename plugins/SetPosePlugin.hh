#ifndef GAZEBO_PLUGINS_SETPOSE_PLUGIN_HH_
#define GAZEBO_PLUGINS_SETPOSE_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
// #include <iginition/math.hh>
namespace gazebo
{
  class SetPosePlugin : public ModelPlugin
  {

  public: SetPosePlugin();
  public: virtual ~SetPosePlugin();
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
  public: void Update();

  //Store pointer to the model
  private: physics::ModelPtr model;
  private: event::ConnectionPtr updateConnection;
  public: std::mutex mutex;
  public: bool flag = false;

  /// Ros stuff:
  private: ros::NodeHandle nh;
  private: ros::Subscriber pose_sub;
  public: void Callback(const geometry_msgs::Pose::ConstPtr& msg);
  /// Initilializing variables to store pose
  private: float x, y, z, roll, pitch, yaw;
};
}
#endif
