#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <optional>
#include <atomic>
#include <string>
#include <cmath>

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TwistStamped;


static void quatErrorAxisAngle(const geometry_msgs::msg::Quaternion &q_curr_msg,
                               const geometry_msgs::msg::Quaternion &q_des_msg,
                               double &angle_out,
                               double axis_out[3])
{
  // Convert to tf2
  tf2::Quaternion q_curr, q_des;
  tf2::fromMsg(q_curr_msg, q_curr);
  tf2::fromMsg(q_des_msg, q_des);

  // Shortest rotation from current -> desired
  // q_err = q_des * q_curr^{-1}
  tf2::Quaternion q_err = q_des * q_curr.inverse();
  q_err.normalize();

  // Extract axis + angle (btQuaternion API)
  double angle = q_err.getAngle();          // in [0, pi]
  tf2::Vector3 axis = q_err.getAxis();      // unit axis (undefined if angle ~ 0)

  // Guard near-zero rotations
  if (!std::isfinite(angle) || angle < 1e-9 || !std::isfinite(axis.x()))
  {
    angle_out = 0.0;
    axis_out[0] = axis_out[1] = axis_out[2] = 0.0;
    return;
  }

  // Fill outputs
  angle_out   = angle;
  axis_out[0] = axis.x();
  axis_out[1] = axis.y();
  axis_out[2] = axis.z();
}

class PoseServoTracker : public rclcpp::Node
{
public:
  PoseServoTracker()
  : Node("pose_servo_tracker"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Parameters
    group_name_    = declare_parameter<std::string>("group_name", "arm");
    ee_link_       = declare_parameter<std::string>("ee_link", "tool0");
    target_frame_  = declare_parameter<std::string>("target_frame", "base_link");
    topic_name_    = declare_parameter<std::string>("target_topic", "/desired_pose");
    servo_twist_topic_ = declare_parameter<std::string>("servo_twist_topic", "/servo_node/delta_twist_cmds");
    enable_topic_      = declare_parameter<std::string>("servo_enable_topic", "/servo_node/enable"); // std_msgs/Bool

    // Gains & deadbands
    kp_lin_        = declare_parameter<double>("kp_linear", 2.0);    // m/s per m error
    kp_ang_        = declare_parameter<double>("kp_angular", 2.0);   // rad/s per rad error
    pos_thr_m_     = declare_parameter<double>("pos_deadband_m", 0.003);
    ang_thr_rad_   = declare_parameter<double>("ang_deadband_rad", 0.02);

    // Saturations
    max_lin_vel_   = declare_parameter<double>("max_linear_vel", 0.25); // m/s
    max_ang_vel_   = declare_parameter<double>("max_angular_vel", 1.0); // rad/s

    loop_hz_       = declare_parameter<double>("loop_rate_hz", 200.0);

    RCLCPP_INFO(get_logger(),
      "Servo tracker: group=%s, ee=%s, frame=%s, target_topic=%s, twist_topic=%s",
      group_name_.c_str(), ee_link_.c_str(), target_frame_.c_str(),
      topic_name_.c_str(), servo_twist_topic_.c_str());

    // Subscribers
    pose_sub_ = create_subscription<PoseStamped>(
      topic_name_, rclcpp::SensorDataQoS(),
      [this](const PoseStamped::SharedPtr msg) { latest_target_pose_ = *msg; });

    // Publishers (to Servo)
    twist_pub_  = create_publisher<TwistStamped>(servo_twist_topic_, 10);
    enable_pub_ = create_publisher<std_msgs::msg::Bool>(enable_topic_, 1);

    // Enable Servo
    std_msgs::msg::Bool en; en.data = true;
    enable_pub_->publish(en);

    // Timer loop
    const auto dt = std::chrono::duration<double>(1.0 / loop_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(dt),
                               std::bind(&PoseServoTracker::timerCallback, this));
  }

private:
  bool getCurrentEEPose(PoseStamped& out) const
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
        "TF lookup %s -> %s failed: %s", target_frame_.c_str(), ee_link_.c_str(), e.what());
      return false;
    }
  }

  void timerCallback()
  {
    if (!latest_target_pose_.has_value())
      return;

    // Transform target into target_frame_ if needed
    PoseStamped target = latest_target_pose_.value();
    try
    {
      if (target.header.frame_id.empty())
      {
        target.header.frame_id = target_frame_;
        target.header.stamp = now();
      }
      if (target.header.frame_id != target_frame_)
      {
        auto tf = tf_buffer_.lookupTransform(target_frame_, target.header.frame_id, tf2::TimePointZero);
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

    // Current EE pose
    PoseStamped current;
    if (!getCurrentEEPose(current)) return;

    // Position error (in target_frame_)
    const double ex = target.pose.position.x - current.pose.position.x;
    const double ey = target.pose.position.y - current.pose.position.y;
    const double ez = target.pose.position.z - current.pose.position.z;
    const double pos_err = std::sqrt(ex*ex + ey*ey + ez*ez);

    // Orientation error
    double angle; double axis[3];
    quatErrorAxisAngle(current.pose.orientation, target.pose.orientation, angle, axis);
    const double ang_err = std::fabs(angle);

    // Deadband check
    if (pos_err < pos_thr_m_ && ang_err < ang_thr_rad_)
    {
      // Send a tiny zero twist to keep Servo happy / watchdog fed
      TwistStamped zero;
      zero.header.stamp = now();
      zero.header.frame_id = target_frame_;  // Twist expressed in planning frame
      twist_pub_->publish(zero);
      return;
    }

    // Proportional control -> twist command
    TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = target_frame_;  // IMPORTANT: must match servo planning_frame

    // Linear (XYZ)
    cmd.twist.linear.x = kp_lin_ * ex;
    cmd.twist.linear.y = kp_lin_ * ey;
    cmd.twist.linear.z = kp_lin_ * ez;

    // Angular (axis * angle)
    cmd.twist.angular.x = kp_ang_ * axis[0] * angle;
    cmd.twist.angular.y = kp_ang_ * axis[1] * angle;
    cmd.twist.angular.z = kp_ang_ * axis[2] * angle;

    // Saturate
    auto clamp = [](double v, double lim){ return std::max(-lim, std::min(lim, v)); };
    cmd.twist.linear.x  = clamp(cmd.twist.linear.x,  max_lin_vel_);
    cmd.twist.linear.y  = clamp(cmd.twist.linear.y,  max_lin_vel_);
    cmd.twist.linear.z  = clamp(cmd.twist.linear.z,  max_lin_vel_);
    cmd.twist.angular.x = clamp(cmd.twist.angular.x, max_ang_vel_);
    cmd.twist.angular.y = clamp(cmd.twist.angular.y, max_ang_vel_);
    cmd.twist.angular.z = clamp(cmd.twist.angular.z, max_ang_vel_);

    twist_pub_->publish(cmd);
  }

private:
  // Params
  std::string group_name_, ee_link_, target_frame_, topic_name_;
  std::string servo_twist_topic_, enable_topic_;
  double kp_lin_{2.0}, kp_ang_{2.0};
  double pos_thr_m_{0.003}, ang_thr_rad_{0.02};
  double max_lin_vel_{0.25}, max_ang_vel_{1.0};
  double loop_hz_{200.0};

  // ROS I/O
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<TwistStamped>::SharedPtr   twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable rclcpp::Clock throttle_clock_{RCL_ROS_TIME};

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // State
  std::optional<PoseStamped> latest_target_pose_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseServoTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
