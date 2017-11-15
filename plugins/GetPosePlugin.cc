#include "GetPosePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(GetPosePlugin);

/////////////////////
//Constructor
GetPosePlugin::GetPosePlugin():ModelPlugin()
{

}

//Destructor
GetPosePlugin::~GetPosePlugin()
{

}

void GetPosePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // store the pointer to the model
  this->model = _parent;
  std::string model_name = this->model->GetName();
  std::string topic_name = model_name + "/" + "getPose" ;
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>(topic_name,1);
  //Listen to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GetPosePlugin::Update, this));
    }


void GetPosePlugin::Update()
{
  auto pos = model->WorldPose().Pos();
  auto rot = model->WorldPose().Rot();
  posedata.header.stamp = ros::Time::now();
  posedata.pose.position.x = pos.X();
  posedata.pose.position.y = pos.Y();
  posedata.pose.position.z = pos.Z();
  posedata.pose.orientation.x = rot.Roll();
  posedata.pose.orientation.y = rot.Pitch();
  posedata.pose.orientation.z = rot.Yaw();
  pose_pub.publish(posedata);
}
