#include "SetPosePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(SetPosePlugin);

/////////////////////
//Constructor
SetPosePlugin::SetPosePlugin():ModelPlugin()
{
  pose_sub = nh.subscribe("SetObjectPose",1 , &SetPosePlugin::Callback, this);

}

//Destructor
SetPosePlugin::~SetPosePlugin()
{

}
void SetPosePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // store the pointer to the model
  this->model = _parent;

  //Listen to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&SetPosePlugin::Update, this));
    }


void SetPosePlugin::Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  x = msg->position.x;
  y = msg->position.y;
  z = msg->position.z;
  roll = msg->orientation.x;
  pitch = msg->orientation.y;
  yaw = msg->orientation.z;
  this->flag = true; // FLag to enable mutex
}

void SetPosePlugin::Update()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if(this->flag)
  {
    model->SetWorldPose(ignition::math::Pose3d(x, y, z, roll, pitch, yaw));

  }
}
