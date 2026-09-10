#include <cmath>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace l150_odometry {

// 将 L150 底盘回传的瞬时速度 (TwistStamped) 积分成里程计 (Odometry)。
//
// 注意：姿态/IMU 数据不在此节点处理，而是由外接 Wheeltec N100 mini 驱动
// (fdilink_ahrs) 发布到 /imu。本节点只负责“速度 -> 位姿”的航迹推算，
// 输出 /wheel_odom 供 robot_localization (EKF) 融合。
class L150OdometryNode {
 public:
  L150OdometryNode() : private_nh_("~") {}

  bool Initialize() {
    private_nh_.param("velocity_topic", velocity_topic_,
                      std::string("/l150/velocity"));
    private_nh_.param("odom_topic", odom_topic_,
                      std::string("/wheel_odom"));
    private_nh_.param("odom_frame", odom_frame_, std::string("odom"));
    private_nh_.param("base_frame", base_frame_, std::string("base_link"));
    private_nh_.param("publish_tf", publish_tf_, false);

    private_nh_.param("pose_covariance_xy", pose_covariance_xy_, 0.05);
    private_nh_.param("pose_covariance_yaw", pose_covariance_yaw_, 0.05);
    private_nh_.param("twist_covariance_xy", twist_covariance_xy_, 0.05);
    private_nh_.param("twist_covariance_yaw", twist_covariance_yaw_, 0.05);
    private_nh_.param("max_dt", max_dt_, 0.5);

    if (max_dt_ <= 0.0) {
      ROS_ERROR("~max_dt must be positive");
      return false;
    }

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    velocity_sub_ = nh_.subscribe(
        velocity_topic_, 10, &L150OdometryNode::VelocityCallback, this);

    ROS_INFO("L150 odometry: %s -> %s (%s -> %s), publish_tf=%s",
             velocity_topic_.c_str(), odom_topic_.c_str(),
             odom_frame_.c_str(), base_frame_.c_str(),
             publish_tf_ ? "true" : "false");
    return true;
  }

 private:
  static double NormalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    const double vx = msg->twist.linear.x;
    const double wz = msg->twist.angular.z;

    if (!std::isfinite(vx) || !std::isfinite(wz)) {
      ROS_WARN_THROTTLE(1.0, "Ignored non-finite velocity");
      return;
    }

    const ros::Time stamp =
        msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    if (!initialized_) {
      Reset(stamp);
      Publish(stamp, vx, wz);
      return;
    }

    const double dt = (stamp - last_stamp_).toSec();
    if (dt < 0.0) {
      ROS_WARN_THROTTLE(1.0, "Time moved backwards; resetting odometry");
      Reset(stamp);
      Publish(stamp, vx, wz);
      return;
    }

    if (dt > max_dt_) {
      // 数据长时间断流：只更新时间基准，不做大跨度积分，避免跳变。
      ROS_WARN_THROTTLE(1.0, "Skipped %.3f s integration gap", dt);
      last_stamp_ = stamp;
      Publish(stamp, vx, wz);
      return;
    }

    if (dt > 0.0) {
      // 中点法积分，比直接累加在转弯时更精确。
      const double dtheta = wz * dt;
      x_ += vx * std::cos(yaw_ + 0.5 * dtheta) * dt;
      y_ += vx * std::sin(yaw_ + 0.5 * dtheta) * dt;
      yaw_ = NormalizeAngle(yaw_ + dtheta);
      last_stamp_ = stamp;
    }

    Publish(stamp, vx, wz);
  }

  void Reset(const ros::Time& stamp) {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    last_stamp_ = stamp;
    initialized_ = true;
  }

  void Publish(const ros::Time& stamp, double vx, double wz) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.z = std::sin(0.5 * yaw_);
    odom.pose.pose.orientation.w = std::cos(0.5 * yaw_);

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = wz;

    // 6x6 协方差矩阵，对角线索引为 0, 7, 14, 21, 28, 35。
    // 未观测的自由度（z、roll、pitch）给大方差，表示 EKF 不应信任。
    odom.pose.covariance[0] = pose_covariance_xy_;
    odom.pose.covariance[7] = pose_covariance_xy_;
    odom.pose.covariance[14] = 1.0e6;
    odom.pose.covariance[21] = 1.0e6;
    odom.pose.covariance[28] = 1.0e6;
    odom.pose.covariance[35] = pose_covariance_yaw_;

    odom.twist.covariance[0] = twist_covariance_xy_;
    odom.twist.covariance[7] = twist_covariance_xy_;
    odom.twist.covariance[14] = 1.0e6;
    odom.twist.covariance[21] = 1.0e6;
    odom.twist.covariance[28] = 1.0e6;
    odom.twist.covariance[35] = twist_covariance_yaw_;

    odom_pub_.publish(odom);

    if (publish_tf_) {
      geometry_msgs::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = x_;
      transform.transform.translation.y = y_;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_.sendTransform(transform);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber velocity_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  std::string velocity_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{false};

  double pose_covariance_xy_{0.05};
  double pose_covariance_yaw_{0.05};
  double twist_covariance_xy_{0.05};
  double twist_covariance_yaw_{0.05};
  double max_dt_{0.5};

  bool initialized_{false};
  ros::Time last_stamp_;
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
};

}  // namespace l150_odometry

int main(int argc, char** argv) {
  ros::init(argc, argv, "l150_odometry_node");
  l150_odometry::L150OdometryNode node;
  if (!node.Initialize()) {
    return 1;
  }
  ros::spin();
  return 0;
}
