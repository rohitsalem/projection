#include "GetPosePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(GetPosePlugin);

/////////////////////
//Constructor
GetPosePlugin::GetPosePlugin():ModelPlugin()
{
    
  	pose_pub = nh.advertise<std_msgs::Float64MultiArray>("getpose",1);

}

//Destructor
GetPosePlugin::~GetPosePlugin()
{

}
void GetPosePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // store the pointer to the model
  this->model = _parent;

  //Listen to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GetPosePlugin::Update, this));
    }


void GetPosePlugin::Update()
{

  auto pos = model->WorldPose().Pos();
  auto rot = model->WorldPose().Rot();
  posedata.data.clear(); //clear the contents of the array
  posedata.data.push_back(pos.X()); // Appending the world pose to the same posedata array
	posedata.data.push_back(pos.Y());
	posedata.data.push_back(pos.Z());
	posedata.data.push_back(rot.Roll());
	posedata.data.push_back(rot.Pitch());
	posedata.data.push_back(rot.Yaw());

  pose_pub.publish(posedata);



}
