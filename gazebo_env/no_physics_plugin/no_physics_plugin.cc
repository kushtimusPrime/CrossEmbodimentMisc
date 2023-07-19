#ifndef _NOPHYSICS_PLUGIN_HH_
#define _NOPHYSICS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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
      // Initialize ROS node
      this->node_ = gazebo_ros::Node::Get(_sdf);
      const gazebo_ros::QoS &qos = this->node_->get_qos();
      // Just output a message for now
      std::cerr << "NO PHYSICS" << "\n";
      _model->GetWorld()->SetPhysicsEnabled(false);
      this->chatter_subscriber_ = this->node_->create_subscription<std_msgs::msg::String>(
                "chatter",
                qos.get_subscription_qos("chatter", rclcpp::QoS(1)),
                std::bind(&NoPhysicsPlugin::ChatterMsg, this, std::placeholders::_1));
    }

    void ChatterMsg(const std_msgs::msg::String::SharedPtr msg) {
        std::cout << "COOKING" << std::endl;
    }


    private:
      gazebo_ros::Node::SharedPtr node_;
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chatter_subscriber_;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(NoPhysicsPlugin)
}
#endif
