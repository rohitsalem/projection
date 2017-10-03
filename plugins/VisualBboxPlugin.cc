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
	public: ros::NodeHandle nh;

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
	this->dataPtr->pub = this->dataPtr->nh.advertise<std_msgs::Float64MultiArray>("corners",2);
}

VisualBboxPlugin::~VisualBboxPlugin()
{
	this->dataPtr->infosub.reset();
	if(this->dataPtr->node)
		this->dataPtr->node->Fini();
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

void VisualBboxPlugin::Update()
{
	std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

	if(!this->dataPtr->visual)
	{
		gzerr << "The Visual is null" <<std::endl;
		return;
	}
	// shows the BoundingBox of the object in gazebo
	this->dataPtr->visual->ShowBoundingBox();
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
	std::cout << " Min " << x_min << '\n' << y_min << '\n'<< z_min << '\n';
	std::cout << " Max" << x_max << '\n' << y_max << '\n' << z_max << '\n';
	corners.data.clear(); // clear the contents of the array if any
	corners.data.push_back(x_min); // append the corners to the corners array
	corners.data.push_back(y_min);
	corners.data.push_back(z_min);
	corners.data.push_back(x_max);
	corners.data.push_back(y_max);
	corners.data.push_back(z_max);
  this->dataPtr->pub.publish(corners);
}


void VisualBboxPlugin::OnInfo(ConstPosesStampedPtr &_msg)
{
	std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
	this->dataPtr->currentSimtime = msgs::Convert(_msg->time());
}
