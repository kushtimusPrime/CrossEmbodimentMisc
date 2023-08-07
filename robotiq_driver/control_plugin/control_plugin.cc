#ifndef _CONTROL_PLUGIN_HH_
#define _CONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class ControlPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ControlPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->node_ = gazebo_ros::Node::Get(_sdf);
        this->model_name_ = _model->GetName();
        const gazebo_ros::QoS &qos = this->node_->get_qos();
      // Just output a message for now
      std::cerr << "KUSHTIMUS PRIME" << "]\n";
      _model->GetWorld()->SetPhysicsEnabled(false);
      this->joint_subscriber_ = this->node_->create_subscription<std_msgs::msg::Float64>(
                "gripper_command",
                qos.get_subscription_qos("gripper_command", rclcpp::QoS(1)),
                std::bind(&ControlPlugin::jointMsg, this, std::placeholders::_1));
      auto left_knuckle = _model->GetJoint("robotiq_85_left_inner_knuckle_joint");
      std::cout << left_knuckle->Position() << std::endl;
    //   std::cout << left_knuckle->SetPosition(0,msg->data) << std::endl;
    //   std::cout << left_knuckle->Position() << std::endl;
    }

    void jointMsg(const std_msgs::msg::Float64::SharedPtr msg) {
        physics::WorldPtr world = physics::get_world("default");
        auto gripper_model_ptr = world->ModelByName(model_name_);
        auto left_knuckle = gripper_model_ptr->GetJoint("robotiq_85_left_knuckle_joint");
        auto right_knuckle = gripper_model_ptr->GetJoint("robotiq_85_right_knuckle_joint");
        auto left_inner_knuckle = gripper_model_ptr->GetJoint("robotiq_85_left_inner_knuckle_joint");
        auto right_inner_knuckle = gripper_model_ptr->GetJoint("robotiq_85_right_inner_knuckle_joint");
        auto left_finger = gripper_model_ptr->GetJoint("robotiq_85_left_finger_tip_joint");
        auto right_finger = gripper_model_ptr->GetJoint("robotiq_85_right_finger_tip_joint");

        left_knuckle->SetPosition(0,msg->data);
        right_knuckle->SetPosition(0,-msg->data);
        left_inner_knuckle->SetPosition(0,msg->data);
        right_inner_knuckle->SetPosition(0,-msg->data);
        left_finger->SetPosition(0,-msg->data);
        right_finger->SetPosition(0,msg->data);
    }

    private:
        std::string model_name_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_subscriber_;
        gazebo_ros::Node::SharedPtr node_;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)
}
#endif