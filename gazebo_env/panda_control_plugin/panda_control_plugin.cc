#ifndef _PANDACONTROL_PLUGIN_HH_
#define _PANDACONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace gazebo
{
  /// \brief A plugin to control a NoPhysics sensor.
  class PandaControlPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: PandaControlPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Initialize ROS node
      this->node_ = gazebo_ros::Node::Get(_sdf);
      this->model_name_ = _model->GetName();
      const gazebo_ros::QoS &qos = this->node_->get_qos();
      // Just output a message for now
      std::cerr << "KUSHTIMUS PRIME" << "\n";
      this->robot_subscriber_ = this->node_->create_subscription<std_msgs::msg::Float64MultiArray>(
                "joint_commands",
                qos.get_subscription_qos("joint_commands", rclcpp::QoS(1)),
                std::bind(&PandaControlPlugin::jointCommandMsg, this, std::placeholders::_1));
    }

    void jointCommandMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        physics::WorldPtr world = physics::get_world("default");
        auto model_ptr = world->ModelByName(model_name_);
        auto panda_joint1 = model_ptr->GetJoint("panda_joint1");
        auto panda_joint2 = model_ptr->GetJoint("panda_joint2");
        auto panda_joint3 = model_ptr->GetJoint("panda_joint3");
        auto panda_joint4 = model_ptr->GetJoint("panda_joint4");
        auto panda_joint5 = model_ptr->GetJoint("panda_joint5");
        auto panda_joint6 = model_ptr->GetJoint("panda_joint6");
        auto panda_joint7 = model_ptr->GetJoint("panda_joint7");
        auto panda_finger_joint1 = model_ptr->GetJoint("panda_finger_joint1");
        auto panda_finger_joint2 = model_ptr->GetJoint("panda_finger_joint2");
        // Needs to coordinate with custon_joint_state_publisher_node.py
        panda_joint1->SetPosition(0,msg->data[0]);
        panda_joint2->SetPosition(0,msg->data[1]);
        panda_joint3->SetPosition(0,msg->data[2]);
        panda_joint4->SetPosition(0,msg->data[3]);
        panda_joint5->SetPosition(0,msg->data[4]);
        panda_joint6->SetPosition(0,msg->data[5]);
        panda_joint7->SetPosition(0,msg->data[6]);
        panda_finger_joint1->SetPosition(0,msg->data[7]);
        panda_finger_joint2->SetPosition(0,msg->data[7]);
    }


    private:
      gazebo_ros::Node::SharedPtr node_;
      std::string model_name_;
      rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr robot_subscriber_;
      
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(PandaControlPlugin)
}
#endif
