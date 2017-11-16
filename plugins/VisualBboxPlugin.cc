#include <mutex>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>
#include "VisualBboxPlugin.hh"

namespace gazebo
{
	class VisualBboxPluginPrivate
	{

	/// \brief Private data for VisualBboxPlugin class
	public: rendering::VisualPtr visual;


	/// \brief
	public: event:: ConnectionPtr updateConnection;

	/// \brief Time taken by a full cycle.
	public: common::Time period;

	/// \brief Time the current cycle started
	public: common::Time cycleStartTime;

	/// \brief The current simulation time
	public: common::Time currentSimtime;

	/// \brief Gazebo Node used for communication
	public: transport::NodePtr node;

	public: std::mutex mutex;

	/// \brief Gazebo subscriber to world info
	public: transport::SubscriberPtr infosub;

	/// \brief True to use Wall time, false to use sim time
  public: bool useWallTime;

	/// Ros Node handle
	public: ros::NodeHandle *nh = nullptr;

	/// Ros publisher : to publish corners array
	public: ros::Publisher pub;


	};
}


using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(VisualBboxPlugin)

VisualBboxPlugin::VisualBboxPlugin() : dataPtr(new VisualBboxPluginPrivate)
{
	int argc = 0;
	char *argv = nullptr;
	ros::init(argc, &argv, "VisualBboxPlugin");
	this->dataPtr->nh = new ros::NodeHandle();
	this->dataPtr->pub = this->dataPtr->nh->advertise<projection::Float64MultiArrayStamped>("objectBoxWorldCoordinates",1);

}

VisualBboxPlugin::~VisualBboxPlugin()
{
	this->dataPtr->infosub.reset();
	if(this->dataPtr->node)
		this->dataPtr->node->Fini();
	delete this->dataPtr->nh;
}

void VisualBboxPlugin::Load (rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
	if(!_visual || !_sdf)
	{
		gzerr << "No visual or SDF elemet Specified. Plugin won't load"<<
			std::endl;
		return;
	}
	this->dataPtr->visual = _visual;

	// Get wether to use wall time or sim time
	this->dataPtr->useWallTime = false;
	if (_sdf->HasElement("use_wall_time"))
		this->dataPtr->useWallTime = _sdf->Get<bool>("use_wall_time");

	// Connect to the world update signal
	this->dataPtr->updateConnection = event::Events::ConnectPreRender(
		std::bind(&VisualBboxPlugin::Update, this));

	if (!this->dataPtr->useWallTime)
	{
		this->dataPtr->node = transport::NodePtr(new transport::Node());
		this->dataPtr->node->Init();

		this->dataPtr->infosub = this->dataPtr->node->Subscribe(
			"~/pose/local/info", &VisualBboxPlugin::OnInfo,this);
	}
}

void VisualBboxPlugin::computeTransformationMatrix(float r , float p, float y, Vec3f c)
{
	Matrix44f Rx(1, 0, 0, 0, 0, cos(r), -sin(-r), 0, 0, sin(-r), cos(r), 0, 0, 0, 0, 1);
	Matrix44f Ry(cos(p), 0, sin(-p), 0, 0, 1, 0, 0, -sin(-p), 0, cos(p), 0, 0, 0, 0, 1);
	Matrix44f Rz(cos(y), -sin(-y), 0, 0, sin(-y), cos(y), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
	Matrix44f T(1,0,0,0,0,1,0,0,0,0,1,0,c[0],c[1],c[2],1);

	Matrix44f Rxy;
	Matrix44f Rxyz;

	Matrix44f::multiply(Rx,Ry,Rxy); // Saves Rx*Ry in Rxy
	Matrix44f::multiply(Rxy, Rz, Rxyz); // Saves Rxy*Rz in Rxyz
	Matrix44f::multiply( Rxyz, T, RT); // Saves Rxyz*T in RT

}

void VisualBboxPlugin::Update()
{
	if(!this->dataPtr->visual)
	{
		gzerr << "The Visual is null" <<std::endl;
		return;
	}
	// shows the BoundingBox of the object in gazebo
	// this->dataPtr->visual->ShowBoundingBox();
	// returns a box object (ignition::math::Box)
	auto bbox =  this->dataPtr->visual->BoundingBox();
	// to get the minimum corner
	auto min_vec = bbox.Min();
	// to get the maximum corner
	auto max_vec = bbox.Max();

	// Storing the vector in terms of simple variables (double type)
	// min corner of the bbox
 	x_min = min_vec[0];
	y_min = min_vec[1];
	z_min = min_vec[2];
	//max corner of the bbox
	x_max = max_vec[0];
	y_max = max_vec[1];
	z_max = max_vec[2];

	// Vertices of cuboid with respect to the object's origin.
	Vec3f pA_o(x_min, y_min, z_min);
	Vec3f pB_o(-x_min, y_min, z_min);
	Vec3f pC_o(-x_min, -y_min, z_min);
	Vec3f pD_o(x_min, -y_min, z_min);
	Vec3f pE_o(x_max, y_max, z_max);
	Vec3f pF_o(-x_max, y_max, z_max);
	Vec3f pG_o(-x_max, -y_max, z_max);
	Vec3f pH_o(x_max, -y_max, z_max);
	auto pos = this->dataPtr->visual->WorldPose().Pos();
	auto rot = this->dataPtr->visual->WorldPose().Rot();

	x_world = pos.X();
	y_world = pos.Y();
	z_world = pos.Z();
	roll_world = rot.Roll();
	pitch_world = rot.Pitch();
	yaw_world = rot.Yaw();

	// Object's world coordiantes along with roll pitch and yaw :
	Vec3f pO_w (x_world, y_world, z_world);
	VisualBboxPlugin::computeTransformationMatrix(roll_world, pitch_world, yaw_world, pO_w);

	// compute the world coordinates of the corners of the box
	RT.Matrix44f::multVecMatrix(pA_o,pWorldA);
	RT.Matrix44f::multVecMatrix(pB_o,pWorldB);
	RT.Matrix44f::multVecMatrix(pC_o,pWorldC);
	RT.Matrix44f::multVecMatrix(pD_o,pWorldD);
	RT.Matrix44f::multVecMatrix(pE_o,pWorldE);
	RT.Matrix44f::multVecMatrix(pF_o,pWorldF);
	RT.Matrix44f::multVecMatrix(pG_o,pWorldG);
	RT.Matrix44f::multVecMatrix(pH_o,pWorldH);

	// Transfer into data msg to publish
	worldArr.array.data.clear();
	worldArr.array.data.push_back(pWorldA[0]);
	worldArr.array.data.push_back(pWorldA[1]);
	worldArr.array.data.push_back(pWorldA[2]);
	worldArr.array.data.push_back(pWorldB[0]);
	worldArr.array.data.push_back(pWorldB[1]);
	worldArr.array.data.push_back(pWorldB[2]);
	worldArr.array.data.push_back(pWorldC[0]);
	worldArr.array.data.push_back(pWorldC[1]);
	worldArr.array.data.push_back(pWorldC[2]);
	worldArr.array.data.push_back(pWorldD[0]);
	worldArr.array.data.push_back(pWorldD[1]);
	worldArr.array.data.push_back(pWorldD[2]);
	worldArr.array.data.push_back(pWorldE[0]);
	worldArr.array.data.push_back(pWorldE[1]);
	worldArr.array.data.push_back(pWorldE[2]);
	worldArr.array.data.push_back(pWorldF[0]);
	worldArr.array.data.push_back(pWorldF[1]);
	worldArr.array.data.push_back(pWorldF[2]);
	worldArr.array.data.push_back(pWorldG[0]);
	worldArr.array.data.push_back(pWorldG[1]);
	worldArr.array.data.push_back(pWorldG[2]);
	worldArr.array.data.push_back(pWorldH[0]);
	worldArr.array.data.push_back(pWorldH[1]);
	worldArr.array.data.push_back(pWorldH[2]);
	worldArr.header.stamp = ros::Time::now();
	worldArr.header.seq++;
	//Publish message
	this->dataPtr->pub.publish(worldArr);
}


void VisualBboxPlugin::OnInfo(ConstPosesStampedPtr &_msg)
{
	std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
	this->dataPtr->currentSimtime = msgs::Convert(_msg->time());
}
