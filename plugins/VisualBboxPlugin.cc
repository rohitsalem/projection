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
	};
}


using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(VisualBboxPlugin)

VisualBboxPlugin::VisualBboxPlugin() : dataPtr(new VisualBboxPluginPrivate)
{
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
	// auto box = this->dataPtr->visual->BoundingBox();
	// this->dataPtr->visual->GetHighlighted();
	this->dataPtr->visual->ShowBoundingBox();
	auto bbox =  this->dataPtr->visual->BoundingBox();
	// std::cout << "/* message */" << bbox <<'\n';
}


void VisualBboxPlugin::OnInfo(ConstPosesStampedPtr &_msg)
{
	std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
	this->dataPtr->currentSimtime = msgs::Convert(_msg->time());
}
