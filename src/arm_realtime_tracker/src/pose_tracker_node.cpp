#include <memory>
#include <optional>
#include <string>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

using geometry_msgs::msg::PoseStamped;
using moveit::planning_interface::MoveGroupInterface;

static double quatShortestAngle(const geometry_msgs::msg::Quaternion &a,
                                const geometry_msgs::msg::Quaternion &b)
{
  // angle = 2 * acos(|dot(a,b)|)
  const double dot = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
  double d = std::min(1.0, std::max(-1.0, std::fabs(dot)));
  return 2.0 * std::acos(d);
}

class PoseTrackerNode : public rclcpp::Node
{
public:
  PoseTrackerNode()
  : Node("pose_tracker_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Parameters
    group_name_   = this->declare_parameter<std::string>("group_name", "arm");
    ee_link_      = this->declare_parameter<std::string>("ee_link", "link6");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
    topic_name_   = this->declare_parameter<std::string>("target_topic", "/desired_pose");

    velocity_scale_ = this->declare_parameter<double>("velocity_scale", 0.1);
    accel_scale_    = this->declare_parameter<double>("accel_scale", 0.1);
    plan_time_      = this->declare_parameter<double>("planning_time", 0.5);
    exec_period_ms_ = this->declare_parameter<int>("exec_period_ms", 100);

    pos_thr_m_      = this->declare_parameter<double>("deadband_pos_m", 0.005);   // 5mm
    ang_thr_rad_    = this->declare_parameter<double>("deadband_rot_rad", 0.02);  // ~1.15 deg
    stale_js_sec_   = this->declare_parameter<double>("stale_joint_state_sec", 0.5);

    RCLCPP_INFO(get_logger(),
      "Starting with: group='%s', ee_link='%s', target_frame='%s', topic='%s'",
      group_name_.c_str(), ee_link_.c_str(), target_frame_.c_str(), topic_name_.c_str());

    // Subscribe to desired pose
    pose_sub_ = create_subscription<PoseStamped>(
      topic_name_, rclcpp::SensorDataQoS(),
      [this](const PoseStamped::SharedPtr msg) { this->poseCallback(*msg); });

    // Subscribe to /joint_states with SensorDataQoS and track timestamp
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr js)
      {
        if (js->header.stamp.sec == 0 && js->header.stamp.nanosec == 0)
          last_js_stamp_ = this->now();  // fallback if publisher uses zero stamp
        else
          last_js_stamp_ = rclcpp::Time(js->header.stamp);

        have_js_ = true;
      });

    // Timer (planning loop)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(exec_period_ms_),
      std::bind(&PoseTrackerNode::timerCallback, this));
  }

  // Call this ONCE from main() after constructing the node
  void initialize_move_group(const rclcpp::Node::SharedPtr& node_ptr)
  {
    try
    {
      move_group_ = std::make_unique<MoveGroupInterface>(node_ptr, group_name_);

      if (!ee_link_.empty())
        move_group_->setEndEffectorLink(ee_link_);

      move_group_->setMaxVelocityScalingFactor(velocity_scale_);
      move_group_->setMaxAccelerationScalingFactor(accel_scale_);
      move_group_->setPlanningTime(plan_time_);
      move_group_->setPoseReferenceFrame(target_frame_);
      move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
      move_group_->setPlannerId("PTP");
      

      // Keep MoveIt's internal state mirror updated in the background.
      // (We will NOT use it for EE pose; we use TF instead.)
      move_group_->startStateMonitor();

      RCLCPP_INFO(get_logger(),
                  "MoveGroupInterface initialized. Planning frame='%s', EE link='%s'",
                  move_group_->getPlanningFrame().c_str(),
                  move_group_->getEndEffectorLink().c_str());

      init_done_ = true;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface init failed: %s", e.what());
    }
  }

private:
  void poseCallback(const PoseStamped& msg)
  {
    latest_target_pose_ = msg;
    last_pose_stamp_ = now();
  }

  bool freshJointStateAvailable() const
  {
    if (!have_js_) return false;
    const double age = (this->now() - last_js_stamp_).seconds();
    return age <= stale_js_sec_;
  }

  // Get current EE pose from TF (target_frame_ -> ee_link_)
  bool getCurrentEEPoseTF(PoseStamped& out) const
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
    catch (const std::exception& e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 2000,
                           "TF lookup %s -> %s failed: %s",
                           target_frame_.c_str(), ee_link_.c_str(), e.what());
      return false;
    }
  }

  void timerCallback()
  {
    if (!init_done_ || !latest_target_pose_.has_value() || executing_)
      return;

    // Require a recent /joint_states sample
    if (!freshJointStateAvailable())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 2000,
                           "No fresh /joint_states (>%.0f ms); skipping this cycle",
                           stale_js_sec_ * 1000.0);
      return;
    }

    // If we just executed, wait for a newer joint state before replanning
    if (waiting_fresh_after_exec_ && last_js_stamp_ <= exec_completed_js_stamp_)
      return;
    waiting_fresh_after_exec_ = false;

    // Copy the latest target
    PoseStamped target = *latest_target_pose_;

    // Transform target into planning frame if needed
    try
    {
      if (target.header.frame_id.empty())
      {
        target.header.frame_id = target_frame_;
        target.header.stamp = this->now();
      }

      if (target.header.frame_id != target_frame_)
      {
        auto tf = tf_buffer_.lookupTransform(
          target_frame_, target.header.frame_id, tf2::TimePointZero);
        tf2::doTransform(target, target, tf);
        target.header.frame_id = target_frame_;
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 2000,
                           "TF transform of target failed: %s", e.what());
      return;
    }

    // Deadband: compare current EE vs target EE using TF (not MoveIt CSM)
    PoseStamped current;
    if (!getCurrentEEPoseTF(current))
      return;

    const double dx = current.pose.position.x - target.pose.position.x;
    const double dy = current.pose.position.y - target.pose.position.y;
    const double dz = current.pose.position.z - target.pose.position.z;
    const double pos_err = std::sqrt(dx*dx + dy*dy + dz*dz);
    const double ang_err = quatShortestAngle(current.pose.orientation, target.pose.orientation);

    RCLCPP_INFO_THROTTLE(get_logger(), throttle_clock_, 1000,
      "EE error: pos = %.2f mm (thr=%.2f), ang = %.2f deg (thr=%.2f)",
      pos_err*1000.0, pos_thr_m_*1000.0,
      ang_err*180.0/M_PI, ang_thr_rad_*180.0/M_PI);

    if (pos_err < pos_thr_m_ && ang_err < ang_thr_rad_)
      return;  // within tolerance; hold position

    // Plan & execute synchronously
    move_group_->setStartStateToCurrentState();  // fine even if CSM occasionally warns
    move_group_->setPoseTarget(target.pose, ee_link_);

    MoveGroupInterface::Plan plan;
    bool ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), throttle_clock_, 1000,
                           "Planning failed to target pose.");
      move_group_->clearPoseTargets();
      return;
    }

    executing_ = true;
    auto res = move_group_->execute(plan);
    executing_ = false;

    // After execution, wait for the next joint state before considering replans
    exec_completed_js_stamp_ = last_js_stamp_;
    waiting_fresh_after_exec_ = true;

    if (res != moveit::core::MoveItErrorCode::SUCCESS)
      RCLCPP_WARN(get_logger(), "Execution finished with error code %d", int(res.val));

    move_group_->clearPoseTargets();
  }

private:
  // Parameters
  std::string group_name_;
  std::string ee_link_;
  std::string target_frame_;
  std::string topic_name_;
  double velocity_scale_{0.4};
  double accel_scale_{0.4};
  double plan_time_{0.5};
  int exec_period_ms_{100};
  double pos_thr_m_{0.005};
  double ang_thr_rad_{0.02};
  double stale_js_sec_{0.5};
  mutable rclcpp::Clock throttle_clock_{RCL_ROS_TIME};
  // ROS
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // MoveIt
  std::unique_ptr<MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;

  // State
  bool init_done_{false};
  std::atomic<bool> executing_{false};
  std::optional<PoseStamped> latest_target_pose_;

  bool have_js_{false};
  rclcpp::Time last_js_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time exec_completed_js_stamp_{0, 0, RCL_ROS_TIME};
  bool waiting_fresh_after_exec_{false};

  rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PoseTrackerNode>();
  node->initialize_move_group(node);

  // Allow callbacks to run during planning/execution
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
