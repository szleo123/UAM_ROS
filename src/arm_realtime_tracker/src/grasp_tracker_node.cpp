#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

using geometry_msgs::msg::PoseStamped;
using moveit::planning_interface::MoveGroupInterface;

namespace
{
double quatShortestAngle(const geometry_msgs::msg::Quaternion &a,
                         const geometry_msgs::msg::Quaternion &b)
{
  const double dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  const double d = std::min(1.0, std::max(-1.0, std::fabs(dot)));
  return 2.0 * std::acos(d);
}
}  // namespace

class GraspTrackerNode : public rclcpp::Node
{
public:
  GraspTrackerNode()
  : Node("grasp_tracker_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    group_arm_name_      = this->declare_parameter<std::string>("arm_group_name", "arm");
    group_gripper_name_  = this->declare_parameter<std::string>("gripper_group_name", "gripper");
    ee_link_             = this->declare_parameter<std::string>("ee_link", "tool0");
    target_frame_        = this->declare_parameter<std::string>("target_frame", "base_link");
    target_topic_        = this->declare_parameter<std::string>("target_topic", "/desired_pose");

    velocity_scale_      = this->declare_parameter<double>("velocity_scale", 0.2);
    accel_scale_         = this->declare_parameter<double>("accel_scale", 0.2);
    plan_time_           = this->declare_parameter<double>("planning_time", 0.75);
    exec_period_ms_      = this->declare_parameter<int>("exec_period_ms", 100);

    arm_approach_named_target_ = this->declare_parameter<std::string>("arm_approach_named_target", "approach");
    pre_grasp_delay_sec_ = this->declare_parameter<double>("pre_grasp_delay_sec", 0.5);
    target_timeout_sec_  = this->declare_parameter<double>("target_timeout_sec", 0.3);
    stale_js_sec_        = this->declare_parameter<double>("stale_joint_state_sec", 0.5);

    arm_home_named_target_      = this->declare_parameter<std::string>("arm_home_named_target", "zero");
    gripper_close_named_target_ = this->declare_parameter<std::string>("gripper_close_named_target", "close");
    gripper_open_named_target_  = this->declare_parameter<std::string>("gripper_open_named_target", "open");
    gripper_joint_name_         = this->declare_parameter<std::string>("gripper_joint_name", "joint7");
    grasp_detection_threshold_  = this->declare_parameter<double>("grasp_detection_threshold", 0.05);

    pos_thr_m_        = this->declare_parameter<double>("deadband_pos_m", 0.005);
    ang_thr_rad_      = this->declare_parameter<double>("deadband_rot_rad", 0.02);

    RCLCPP_INFO(get_logger(),
      "Starting grasp tracker with arm='%s', gripper='%s', ee_link='%s', target_frame='%s', topic='%s'",
      group_arm_name_.c_str(), group_gripper_name_.c_str(), ee_link_.c_str(),
      target_frame_.c_str(), target_topic_.c_str());

    pose_sub_ = create_subscription<PoseStamped>(
      target_topic_, rclcpp::SensorDataQoS(),
      [this](const PoseStamped::SharedPtr msg) { this->poseCallback(*msg); });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr js) { this->jointStateCallback(*js); });

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(exec_period_ms_),
      std::bind(&GraspTrackerNode::timerCallback, this));
  }

  void initialize_move_groups(const rclcpp::Node::SharedPtr &node_ptr)
  {
    try
    {
      move_group_arm_ = std::make_unique<MoveGroupInterface>(node_ptr, group_arm_name_);
      move_group_arm_->setEndEffectorLink(ee_link_);
      move_group_arm_->setMaxVelocityScalingFactor(velocity_scale_);
      move_group_arm_->setMaxAccelerationScalingFactor(accel_scale_);
      move_group_arm_->setPlanningTime(plan_time_);
      move_group_arm_->setPoseReferenceFrame(target_frame_);
      move_group_arm_->setPlanningPipelineId("pilz_industrial_motion_planner");
      move_group_arm_->setPlannerId("PTP");
      move_group_arm_->startStateMonitor();

      move_group_gripper_ = std::make_unique<MoveGroupInterface>(node_ptr, group_gripper_name_);
      move_group_gripper_->setMaxVelocityScalingFactor(1.0);
      move_group_gripper_->setMaxAccelerationScalingFactor(1.0);
      move_group_gripper_->setPlanningTime(plan_time_);
      move_group_gripper_->startStateMonitor();

      // Cache the nominal close position for evaluation (if available)
      auto close_targets = move_group_gripper_->getNamedTargetValues(gripper_close_named_target_);
      auto joint_it = close_targets.find(gripper_joint_name_);
      if (joint_it != close_targets.end())
        planned_gripper_close_position_ = joint_it->second;

      init_done_ = true;
      RCLCPP_INFO(get_logger(), "MoveGroup interfaces ready. Planned gripper close position=%.3f",
                  planned_gripper_close_position_);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to initialize MoveGroup interfaces: %s", e.what());
    }
  }

private:
  enum class Stage
  {
    WAIT_FOR_TARGET = 0,
    MOVE_TO_APPROACH,
    MOVE_TO_GRASP,
    PRE_GRASP_DELAY,
    CLOSE_GRIPPER,
    RETURN_TO_APPROACH,
    MOVE_HOME,
    EVALUATE_GRASP
  };

  void poseCallback(const PoseStamped &msg)
  {
    latest_target_pose_ = msg;
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0)
      last_pose_stamp_ = now();
    else
      last_pose_stamp_ = rclcpp::Time(msg.header.stamp);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState &js)
  {
    std::scoped_lock lock(joint_mutex_);

    if (js.header.stamp.sec == 0 && js.header.stamp.nanosec == 0)
      last_js_stamp_ = this->now();
    else
      last_js_stamp_ = rclcpp::Time(js.header.stamp);

    joint_positions_.clear();
    for (size_t i = 0; i < js.name.size() && i < js.position.size(); ++i)
      joint_positions_[js.name[i]] = js.position[i];

    have_js_ = true;
  }

  bool freshJointStateAvailable() const
  {
    if (!have_js_)
      return false;

    const double age = (this->now() - last_js_stamp_).seconds();
    return age <= stale_js_sec_;
  }

  std::optional<double> getJointPosition(const std::string &joint) const
  {
    std::scoped_lock lock(joint_mutex_);
    auto it = joint_positions_.find(joint);
    if (it == joint_positions_.end())
      return std::nullopt;
    return it->second;
  }

  bool getCurrentEEPoseTF(PoseStamped &out) const
  {
    try
    {
      auto tf = tf_buffer_.lookupTransform(target_frame_, ee_link_, tf2::TimePointZero);
      out.header.stamp = rclcpp::Time(tf.header.stamp);
      out.header.frame_id = target_frame_;
      out.pose.position.x = tf.transform.translation.x;
      out.pose.position.y = tf.transform.translation.y;
      out.pose.position.z = tf.transform.translation.z;
      out.pose.orientation = tf.transform.rotation;
      return true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 2000,
                           "TF lookup %s -> %s failed: %s",
                           target_frame_.c_str(), ee_link_.c_str(), e.what());
      return false;
    }
  }

  bool latestTargetFresh() const
  {
    if (!latest_target_pose_.has_value())
      return false;
    if (target_timeout_sec_ <= 1e-6)
      return true;
    const double age = (this->now() - last_pose_stamp_).seconds();
    return age <= target_timeout_sec_;
  }

  bool transformTarget(PoseStamped &target) const
  {
    try
    {
      if (target.header.frame_id.empty())
      {
        target.header.frame_id = target_frame_;
        target.header.stamp = this->now();
      }
      if (target.header.frame_id != target_frame_)
      {
        auto tf = tf_buffer_.lookupTransform(target_frame_, target.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(target, target, tf);
        target.header.frame_id = target_frame_;
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 2000,
                           "Failed to transform target: %s", e.what());
      return false;
    }

    tf2::Quaternion q;
    tf2::fromMsg(target.pose.orientation, q);
    if (q.length2() <= std::numeric_limits<double>::epsilon())
      q.setValue(0.0, 0.0, 0.0, 1.0);
    else
      q.normalize();
    target.pose.orientation = tf2::toMsg(q);
    return true;
  }

  bool computeGraspPose(PoseStamped &target_pose)
  {
    if (!latest_target_pose_.has_value())
      return false;
    target_pose = *latest_target_pose_;
    if (!transformTarget(target_pose))
      return false;
    return true;
  }

  bool planAndExecuteArmPose(const PoseStamped &target_pose)
  {
    move_group_arm_->setStartStateToCurrentState();
    if (!move_group_arm_->setPoseTarget(target_pose.pose, ee_link_))
    {
      RCLCPP_WARN(get_logger(), "Failed to set pose target for arm (IK).");
      move_group_arm_->clearPoseTargets();
      return false;
    }

    MoveGroupInterface::Plan plan;
    if (move_group_arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Planning failed for arm.");
      move_group_arm_->clearPoseTargets();
      return false;
    }

    executing_ = true;
    auto exec_res = move_group_arm_->execute(plan);
    executing_ = false;
    move_group_arm_->clearPoseTargets();

    exec_completed_js_stamp_ = last_js_stamp_;
    waiting_fresh_after_exec_ = true;

    if (exec_res != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Arm execution returned error code %d", int(exec_res.val));
      return false;
    }
    return true;
  }

  bool executeGripperNamed(const std::string &named_target)
  {
    if (!move_group_gripper_->setNamedTarget(named_target))
    {
      RCLCPP_WARN(get_logger(), "Failed to set gripper named target '%s'", named_target.c_str());
      return false;
    }

    MoveGroupInterface::Plan plan;
    if (move_group_gripper_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Planning failed for gripper target '%s'", named_target.c_str());
      return false;
    }

    executing_ = true;
    auto exec_res = move_group_gripper_->execute(plan);
    executing_ = false;

    exec_completed_js_stamp_ = last_js_stamp_;
    waiting_fresh_after_exec_ = true;

    if (exec_res != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Gripper execution returned error code %d", int(exec_res.val));
      return false;
    }
    return true;
  }

  void timerCallback()
  {
    if (!init_done_ || executing_)
      return;

    if (!freshJointStateAvailable())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 2000,
                           "No fresh /joint_states (>%.0f ms); holding.", stale_js_sec_ * 1000.0);
      return;
    }

    if (waiting_fresh_after_exec_ && last_js_stamp_ <= exec_completed_js_stamp_)
      return;

    if (waiting_fresh_after_exec_)
      waiting_fresh_after_exec_ = false;

    switch (stage_)
    {
      case Stage::WAIT_FOR_TARGET:
        handleWaitForTarget();
        break;
      case Stage::MOVE_TO_APPROACH:
        handleMoveToApproach();
        break;
      case Stage::MOVE_TO_GRASP:
        handleMoveToGrasp();
        break;
      case Stage::PRE_GRASP_DELAY:
        handlePreGraspDelay();
        break;
      case Stage::CLOSE_GRIPPER:
        handleCloseGripper();
        break;
      case Stage::RETURN_TO_APPROACH:
        handleReturnToApproach();
        break;
      case Stage::MOVE_HOME:
        handleMoveHome();
        break;
      case Stage::EVALUATE_GRASP:
        handleEvaluateGrasp();
        break;
    }
  }

  void handleWaitForTarget()
  {
    if (grasp_completed_)
      return;

    if (!latestTargetFresh())
      return;

    pre_grasp_delay_active_ = false;

    PoseStamped grasp_pose;
    if (!computeGraspPose(grasp_pose))
      return;

    stage_ = Stage::MOVE_TO_APPROACH;
    RCLCPP_INFO(get_logger(), "Stage -> MOVE_TO_APPROACH");
  }

  void handleMoveToApproach()
  {
    if (!executeArmNamed(arm_approach_named_target_))
    {
      pre_grasp_delay_active_ = false;
      stage_ = Stage::WAIT_FOR_TARGET;
      grasp_completed_ = false;
      return;
    }

    stage_ = Stage::MOVE_TO_GRASP;
    RCLCPP_INFO(get_logger(), "Stage -> MOVE_TO_GRASP");
  }

  void handleMoveToGrasp()
  {
    if (!latestTargetFresh())
    {
      pre_grasp_delay_active_ = false;
      stage_ = Stage::WAIT_FOR_TARGET;
      grasp_completed_ = false;
      return;
    }

    PoseStamped grasp_pose;
    if (!computeGraspPose(grasp_pose))
    {
      pre_grasp_delay_active_ = false;
      stage_ = Stage::WAIT_FOR_TARGET;
      grasp_completed_ = false;
      return;
    }

    PoseStamped current_pose;
    if (getCurrentEEPoseTF(current_pose))
    {
      const double dx = current_pose.pose.position.x - grasp_pose.pose.position.x;
      const double dy = current_pose.pose.position.y - grasp_pose.pose.position.y;
      const double dz = current_pose.pose.position.z - grasp_pose.pose.position.z;
      const double pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);
      const double ang_err = quatShortestAngle(current_pose.pose.orientation, grasp_pose.pose.orientation);

      if (pos_err < pos_thr_m_ && ang_err < ang_thr_rad_)
      {
        RCLCPP_INFO(get_logger(), "Already at grasp pose tolerance (%.1f mm, %.1f deg).",
                    pos_err * 1000.0, ang_err * 180.0 / M_PI);
        pre_grasp_delay_active_ = false;
        stage_ = Stage::PRE_GRASP_DELAY;
        RCLCPP_INFO(get_logger(), "Stage -> PRE_GRASP_DELAY");
        return;
      }
    }

    if (!planAndExecuteArmPose(grasp_pose))
    {
      pre_grasp_delay_active_ = false;
      stage_ = Stage::WAIT_FOR_TARGET;
      grasp_completed_ = false;
      return;
    }

    pre_grasp_delay_active_ = false;
    stage_ = Stage::PRE_GRASP_DELAY;
    RCLCPP_INFO(get_logger(), "Stage -> PRE_GRASP_DELAY");
  }

  void handlePreGraspDelay()
  {
    if (pre_grasp_delay_sec_ <= 1e-6)
    {
      stage_ = Stage::CLOSE_GRIPPER;
      return;
    }

    if (!pre_grasp_delay_active_)
    {
      pre_grasp_delay_active_ = true;
      pre_grasp_delay_start_ = this->now();
      RCLCPP_INFO(get_logger(), "Pre-grasp delay started (%.2f s).", pre_grasp_delay_sec_);
      return;
    }

    const double elapsed = (this->now() - pre_grasp_delay_start_).seconds();
    if (elapsed >= pre_grasp_delay_sec_)
    {
      pre_grasp_delay_active_ = false;
      stage_ = Stage::CLOSE_GRIPPER;
      RCLCPP_INFO(get_logger(), "Pre-grasp delay finished.");
    }
  }

  void handleCloseGripper()
  {
    if (!executeGripperNamed(gripper_close_named_target_))
    {
      pre_grasp_delay_active_ = false;
      stage_ = Stage::WAIT_FOR_TARGET;
      grasp_completed_ = false;
      return;
    }

    stage_ = Stage::RETURN_TO_APPROACH;
    RCLCPP_INFO(get_logger(), "Stage -> RETURN_TO_APPROACH");
  }

  void handleReturnToApproach()
  {
    if (!executeArmNamed(arm_approach_named_target_))
    {
      pre_grasp_delay_active_ = false;
      stage_ = Stage::WAIT_FOR_TARGET;
      grasp_completed_ = false;
      return;
    }

    stage_ = Stage::EVALUATE_GRASP;
    RCLCPP_INFO(get_logger(), "Stage -> EVALUATE_GRASP");
  }

  void handleMoveHome()
  {
    if (!executeArmNamed(arm_home_named_target_))
    {
      pre_grasp_delay_active_ = false;
      stage_ = Stage::WAIT_FOR_TARGET;
      grasp_completed_ = false;
      return;
    }

    RCLCPP_INFO(get_logger(), "Reached home named target '%s'.", arm_home_named_target_.c_str());
    latest_target_pose_.reset();
    pre_grasp_delay_active_ = false;
    stage_ = Stage::WAIT_FOR_TARGET;
    RCLCPP_INFO(get_logger(), "Stage -> WAIT_FOR_TARGET");
  }

  void handleEvaluateGrasp()
  {
    auto joint_pos_opt = getJointPosition(gripper_joint_name_);
    if (!joint_pos_opt.has_value())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 1000,
                           "Gripper joint '%s' not present in /joint_states yet.",
                           gripper_joint_name_.c_str());
      return;
    }

    const double actual = joint_pos_opt.value();
    const double planned = planned_gripper_close_position_;
    const double diff = std::fabs(planned - actual);

    RCLCPP_INFO(get_logger(), "Gripper planned=%.3f, actual=%.3f, diff=%.3f (thr=%.3f)",
                planned, actual, diff, grasp_detection_threshold_);

    if (diff >= grasp_detection_threshold_)
    {
      RCLCPP_INFO(get_logger(), "Object detected in gripper. Moving to '%s'.",
                  arm_home_named_target_.c_str());
      pre_grasp_delay_active_ = false;
      grasp_completed_ = true;
      stage_ = Stage::MOVE_HOME;
      RCLCPP_INFO(get_logger(), "Stage -> MOVE_HOME");
      return;
    }

    RCLCPP_WARN(get_logger(), "No object detected (diff=%.3f < thr). Re-opening gripper and retrying.", diff);

    if (!executeGripperNamed(gripper_open_named_target_))
    {
      RCLCPP_WARN(get_logger(), "Failed to re-open gripper; staying in WAIT_FOR_TARGET.");
      pre_grasp_delay_active_ = false;
      stage_ = Stage::WAIT_FOR_TARGET;
      grasp_completed_ = false;
      return;
    }

    pre_grasp_delay_active_ = false;
    grasp_completed_ = false;
    stage_ = Stage::MOVE_TO_GRASP;
    RCLCPP_INFO(get_logger(), "Stage -> MOVE_TO_GRASP");
  }

  bool executeArmNamed(const std::string &named_target)
  {
    if (!move_group_arm_->setNamedTarget(named_target))
    {
      RCLCPP_WARN(get_logger(), "Failed to set arm named target '%s'", named_target.c_str());
      return false;
    }

    MoveGroupInterface::Plan plan;
    if (move_group_arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Planning failed for arm named target '%s'", named_target.c_str());
      return false;
    }

    executing_ = true;
    auto exec_res = move_group_arm_->execute(plan);
    executing_ = false;

    exec_completed_js_stamp_ = last_js_stamp_;
    waiting_fresh_after_exec_ = true;

    if (exec_res != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Arm execution returned error code %d", int(exec_res.val));
      return false;
    }
    return true;
  }

private:
  // Parameters / configuration
  std::string group_arm_name_;
  std::string group_gripper_name_;
  std::string ee_link_;
  std::string target_frame_;
  std::string target_topic_;
  double velocity_scale_{0.2};
  double accel_scale_{0.2};
  double plan_time_{0.75};
  int exec_period_ms_{100};
  std::string arm_approach_named_target_{"approach"};
  double pre_grasp_delay_sec_{0.5};
  double target_timeout_sec_{0.3};
  double stale_js_sec_{0.5};
  std::string arm_home_named_target_{"zero"};
  std::string gripper_close_named_target_{"close"};
  std::string gripper_open_named_target_{"open"};
  std::string gripper_joint_name_{"joint7"};
  double grasp_detection_threshold_{0.00};
  double planned_gripper_close_position_{-0.69};
  double pos_thr_m_{0.005};
  double ang_thr_rad_{0.02};

  mutable rclcpp::Clock throttle_clock_{RCL_ROS_TIME};

  // ROS interfaces
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // MoveIt
  std::unique_ptr<MoveGroupInterface> move_group_arm_;
  std::unique_ptr<MoveGroupInterface> move_group_gripper_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  // State tracking
  Stage stage_{Stage::WAIT_FOR_TARGET};
  std::optional<PoseStamped> latest_target_pose_;
  bool init_done_{false};
  std::atomic<bool> executing_{false};

  bool have_js_{false};
  mutable std::mutex joint_mutex_;
  std::unordered_map<std::string, double> joint_positions_;
  rclcpp::Time last_js_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time exec_completed_js_stamp_{0, 0, RCL_ROS_TIME};
  bool waiting_fresh_after_exec_{false};
  bool pre_grasp_delay_active_{false};
  bool grasp_completed_{false};
  rclcpp::Time pre_grasp_delay_start_{0, 0, RCL_ROS_TIME};

  rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GraspTrackerNode>();
  node->initialize_move_groups(node);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
