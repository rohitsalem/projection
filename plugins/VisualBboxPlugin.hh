#ifndef GAZEBO_VISUALBBOXPLUGIN_H
#define GAZEBO_VISUALBBOXPLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <memory>
namespace gazebo
{

	class VisualBboxPluginPrivate;

	class GAZEBO_VISIBLE VisualBboxPlugin : public VisualPlugin
 	{
	
	/// \brief Constructor
	public: VisualBboxPlugin();

	/// \brief Destructor
	public: ~VisualBboxPlugin();

	public: virtual void Load(rendering::VisualPtr _visual,
		sdf::ElementPtr _sdf);

	/// \brief Update the plugin once every iteration of simulation
	private: void Update();


	/// \brief	Callback to receive info.
	private: void OnInfo(ConstPosesStampedPtr &_msg);

	/// \internal 
	/// \brief Private Data pointer 
	private: std::unique_ptr<VisualBboxPluginPrivate> dataPtr;

	};
}

#endif