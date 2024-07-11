
#ifndef ODOMETER_BASE_HPP_
#define ODOMETER_BASE_HPP_

#include <memory>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>

namespace viso2_ros
{

/**
 * Base class for odometers, handles tf's, odometry and pose
 * publishing. This can be used as base for any incremental pose estimating
 * sensor. Sensors that measure velocities cannot be used.
 */
class OdometerBase : public rclcpp::Node
{
private:

  // publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;

  // tf related
  std::string sensor_frame_id_;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  bool publish_tf_;
  bool invert_tf_;

  // the current integrated camera pose
  tf2::Transform integrated_pose_;
  // timestamp of the last update
  rclcpp::Time last_update_time_;

  // covariances
  std::array<double, 36> pose_covariance_;
  std::array<double, 36> twist_covariance_;

public:

  OdometerBase(const std::string& name)
  : Node(name)
  {
    using namespace std::placeholders;
    odom_frame_id_ = this->declare_parameter("odom_frame_id", std::string("/odom"));
    base_link_frame_id_ = this->declare_parameter("base_link_frame_id", std::string("/base_link"));
    sensor_frame_id_ = this->declare_parameter("sensor_frame_id", std::string("/camera"));
    publish_tf_ = this->declare_parameter("publish_tf", true);
    invert_tf_ = this->declare_parameter("invert_tf", false);

    RCLCPP_INFO_STREAM(this->get_logger(), "Basic Odometer Settings:" << std::endl <<
      "  odom_frame_id      = " << odom_frame_id_ << std::endl <<
      "  base_link_frame_id = " << base_link_frame_id_ << std::endl <<
      "  publish_tf         = " << (publish_tf_? "true" : "false") << std::endl <<
      "  invert_tf          = " << (invert_tf_? "true" : "false"));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    reset_service_ = this->create_service<std_srvs::srv::Empty>("reset_pose", 
      std::bind(&OdometerBase::resetPose, this, _1, _2));

    integrated_pose_.setIdentity();
    last_update_time_ = this->get_clock()->now();

    pose_covariance_.fill(0.0);
    twist_covariance_.fill(0.0);
  }

protected:

  void setSensorFrameId(const std::string& frame_id)
  {
    sensor_frame_id_ = frame_id;
  }

  std::string getSensorFrameId() const
  {
    return sensor_frame_id_;
  }

  void setPoseCovariance(const std::array<double, 36>& pose_covariance)
  {
    pose_covariance_ = pose_covariance;
  }
  
  void setTwistCovariance(const std::array<double, 36>& twist_covariance)
  {
    twist_covariance_ = twist_covariance;
  }


  void integrateAndPublish(const tf2::Transform& delta_transform, const rclcpp::Time& timestamp)
  {
    if (sensor_frame_id_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "[odometer] update called with unknown sensor frame id!");
      return;
    }
    if (timestamp < last_update_time_)
    {
      RCLCPP_WARN(this->get_logger(), "[odometer] saw negative time change in incoming sensor data, resetting pose.");
      integrated_pose_.setIdentity();
      //tf_listener_.clear();
    }
    integrated_pose_ *= delta_transform;

    // transform integrated pose to base frame
    geometry_msgs::msg::TransformStamped b2s;
    std::string error_msg;
    
    try {
      b2s = tf_buffer_->lookupTransform(
        base_link_frame_id_, 
        sensor_frame_id_, 
        timestamp);
    } catch (tf2::TransformException &ex) {
      auto& clk = *this->get_clock();
      RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 10.0, 
         "The tf from '%s' to '%s' does not seem to be available, "
         "will assume it as identity!",
         base_link_frame_id_.c_str(),
         sensor_frame_id_.c_str());
    }
    
    // geometry_msg to tf2 
    tf2::Transform base_to_sensor;
    tf2::fromMsg(b2s.transform, base_to_sensor);
    
    tf2::Transform base_transform = base_to_sensor * integrated_pose_ * base_to_sensor.inverse();

    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.header.stamp = timestamp;
    odometry_msg.header.frame_id = odom_frame_id_;
    odometry_msg.child_frame_id = base_link_frame_id_;
    odometry_msg.pose.pose.position.x = base_transform.getOrigin().getX();
    odometry_msg.pose.pose.position.y = base_transform.getOrigin().getY();
    odometry_msg.pose.pose.position.z = base_transform.getOrigin().getZ();
    odometry_msg.pose.pose.orientation = tf2::toMsg(base_transform.getRotation());

    // calculate twist (not possible for first run as no delta_t can be computed)
    tf2::Transform delta_base_transform = base_to_sensor * delta_transform * base_to_sensor.inverse();
    if (last_update_time_.seconds())
    {
      double delta_t = (timestamp - last_update_time_).seconds();
      if (delta_t)
      {
        odometry_msg.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / delta_t;
        odometry_msg.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / delta_t;
        odometry_msg.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / delta_t;
        tf2::Quaternion delta_rot = delta_base_transform.getRotation();
        tf2Scalar angle = delta_rot.getAngle();
        tf2::Vector3 axis = delta_rot.getAxis();
        tf2::Vector3 angular_twist = axis * angle / delta_t;
        odometry_msg.twist.twist.angular.x = angular_twist.x();
        odometry_msg.twist.twist.angular.y = angular_twist.y();
        odometry_msg.twist.twist.angular.z = angular_twist.z();
      }
    }

    odometry_msg.pose.covariance = pose_covariance_;
    odometry_msg.twist.covariance = twist_covariance_;
    odom_pub_->publish(odometry_msg);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = odometry_msg.header.stamp;
    pose_msg.header.frame_id = odometry_msg.header.frame_id;
    pose_msg.pose = odometry_msg.pose.pose;

    pose_pub_->publish(pose_msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = timestamp;
      if (invert_tf_) {
        t.header.frame_id = odom_frame_id_;
        t.child_frame_id = base_link_frame_id_;
        t.transform = tf2::toMsg(base_transform.inverse());      
      } else {
        t.header.frame_id = base_link_frame_id_;
        t.child_frame_id = odom_frame_id_;
        t.transform = tf2::toMsg(base_transform);
      }
      tf_broadcaster_->sendTransform(t);
    }

    last_update_time_ = timestamp;
  }

  void resetPose(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request, 
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    (void) request;
    (void) response;
    integrated_pose_.setIdentity();
  }
};

} // end of namespace

#endif

