#ifndef _NOPHYSICS_PLUGIN_HH_
#define _NOPHYSICS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  /// \brief A plugin to control a NoPhysics sensor.
  class NoPhysicsPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: NoPhysicsPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Just output a message for now
      std::cerr << "NO PHYSICS" << "\n";
      _model->GetWorld()->SetPhysicsEnabled(false);
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(NoPhysicsPlugin)
}
#endif
