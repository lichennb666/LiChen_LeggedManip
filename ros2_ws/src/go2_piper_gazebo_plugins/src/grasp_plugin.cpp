#include <gazebo/gazebo.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <map>
#include <mutex>
#include <string>

namespace go2_piper_gazebo_plugins
{
class GraspPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    robot_ = model;
    world_ = model->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);
    target_model_name_ = SdfString(sdf, "target_model", "target_box");
    target_link_name_ = SdfString(sdf, "target_link", "handle");
    parent_link_name_ = SdfString(sdf, "parent_link", "Link6");
    left_finger_name_ = SdfString(sdf, "left_finger", "Link7");
    right_finger_name_ = SdfString(sdf, "right_finger", "Link8");
    // ContactManager otherwise drops contacts that have no transport
    // subscribers, which would make the service miss valid finger contacts.
    auto manager = world_->Physics()->GetContactManager();
    manager->SetNeverDropContacts(true);

    const auto target = world_->ModelByName(target_model_name_);
    const auto handle = target ? target->GetLink(target_link_name_) : nullptr;
    const auto left = robot_->GetLink(left_finger_name_);
    const auto right = robot_->GetLink(right_finger_name_);
    if (handle && left && right && !handle->GetCollisions().empty() &&
      !left->GetCollisions().empty() && !right->GetCollisions().empty())
    {
      std::map<std::string, gazebo::physics::CollisionPtr> collisions;
      for (const auto & collision : handle->GetCollisions()) {
        collisions[collision->GetScopedName()] = collision;
      }
      for (const auto & collision : left->GetCollisions()) {
        collisions[collision->GetScopedName()] = collision;
      }
      for (const auto & collision : right->GetCollisions()) {
        collisions[collision->GetScopedName()] = collision;
      }
      transport_node_.reset(new gazebo::transport::Node());
      transport_node_->Init(world_->Name());
      const auto topic = manager->CreateFilter(
        "go2_piper_grasp_contacts", collisions);
      contact_subscriber_ = transport_node_->Subscribe(
        topic, &GraspPlugin::OnContacts, this);
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "gripper contact filter is not available yet; attach service will fall back to contact-manager polling");
    }

    attach_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/go2_piper/grasp/attach",
      std::bind(&GraspPlugin::Attach, this, std::placeholders::_1, std::placeholders::_2));
    detach_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/go2_piper/grasp/detach",
      std::bind(&GraspPlugin::Detach, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      node_->get_logger(),
      "Double-contact fixed-joint grasp ready: %s + %s -> %s::%s",
      left_finger_name_.c_str(), right_finger_name_.c_str(),
      target_model_name_.c_str(), target_link_name_.c_str());
  }

private:
  static std::string SdfString(
    const sdf::ElementPtr & sdf, const std::string & key,
    const std::string & fallback)
  {
    return sdf->HasElement(key) ? sdf->Get<std::string>(key) : fallback;
  }

  bool HasFingerHandleContact(const std::string & finger) const
  {
    auto manager = world_->Physics()->GetContactManager();
    const auto count = manager->GetContactCount();
    const std::string handle_scope =
      target_model_name_ + "::" + target_link_name_ + "::";
    for (unsigned int i = 0; i < count; ++i) {
      const auto * contact = manager->GetContact(i);
      if (!contact || !contact->collision1 || !contact->collision2) {
        continue;
      }
      const auto a = contact->collision1->GetScopedName();
      const auto b = contact->collision2->GetScopedName();
      const bool finger_hit =
        a.find(finger) != std::string::npos ||
        b.find(finger) != std::string::npos;
      const bool handle_hit =
        a.find(handle_scope) != std::string::npos ||
        b.find(handle_scope) != std::string::npos;
      if (finger_hit && handle_hit) {
        return true;
      }
    }
    return false;
  }

  void OnContacts(
    const boost::shared_ptr<const gazebo::msgs::Contacts> & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double now = world_->SimTime().Double();
    const std::string handle_scope =
      target_model_name_ + "::" + target_link_name_ + "::";
    for (int i = 0; i < message->contact_size(); ++i) {
      const auto & contact = message->contact(i);
      const auto & a = contact.collision1();
      const auto & b = contact.collision2();
      if (a.find(handle_scope) == std::string::npos &&
        b.find(handle_scope) == std::string::npos)
      {
        continue;
      }
      if (a.find(left_finger_name_) != std::string::npos ||
        b.find(left_finger_name_) != std::string::npos)
      {
        last_left_contact_time_ = now;
      }
      if (a.find(right_finger_name_) != std::string::npos ||
        b.find(right_finger_name_) != std::string::npos)
      {
        last_right_contact_time_ = now;
      }
    }
  }

  void Attach(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fixed_joint_) {
      response->success = true;
      response->message = "fixed joint already attached";
      return;
    }

    const auto target = world_->ModelByName(target_model_name_);
    const auto parent = robot_->GetLink(parent_link_name_);
    const auto handle = target ? target->GetLink(target_link_name_) : nullptr;
    if (!target || !parent || !handle) {
      response->success = false;
      response->message =
        "missing parent link or target_box/handle link";
      return;
    }

    // ROS service callbacks do not run in the physics update thread.  Cache
    // the most recent physics-step contacts so a request cannot miss the
    // short ContactManager snapshot between simulation iterations.
    constexpr double contact_window = 0.25;
    const double now = world_->SimTime().Double();
    const bool left_contact = now - last_left_contact_time_ <= contact_window;
    const bool right_contact = now - last_right_contact_time_ <= contact_window;
    if (!left_contact || !right_contact) {
      response->success = false;
      response->message =
        "bilateral handle contact required (left=" +
        std::string(left_contact ? "true" : "false") + ", right=" +
        std::string(right_contact ? "true" : "false") + ")";
      return;
    }

    fixed_joint_ = world_->Physics()->CreateJoint("fixed", robot_);
    if (!fixed_joint_) {
      response->success = false;
      response->message = "physics engine failed to create fixed joint";
      return;
    }
    fixed_joint_->SetName("go2_piper_assisted_grasp_fixed_joint");
    fixed_joint_->Load(parent, handle, ignition::math::Pose3d::Zero);
    fixed_joint_->Init();

    target_ = target;
    response->success = true;
    response->message =
      "fixed joint attached after Link7+Link8 handle contact";
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
  }

  void Detach(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fixed_joint_) {
      fixed_joint_->Detach();
      fixed_joint_->Fini();
      fixed_joint_.reset();
    }
    target_.reset();
    response->success = true;
    response->message = "fixed grasp joint detached";
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());
  }

  gazebo::physics::ModelPtr robot_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr target_;
  gazebo::physics::JointPtr fixed_joint_;
  gazebo::transport::NodePtr transport_node_;
  gazebo::transport::SubscriberPtr contact_subscriber_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attach_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detach_service_;
  std::string target_model_name_;
  std::string target_link_name_;
  std::string parent_link_name_;
  std::string left_finger_name_;
  std::string right_finger_name_;
  mutable std::mutex mutex_;
  double last_left_contact_time_ = -1.0e9;
  double last_right_contact_time_ = -1.0e9;
};

GZ_REGISTER_MODEL_PLUGIN(GraspPlugin)
}  // namespace go2_piper_gazebo_plugins
