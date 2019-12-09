#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <collision_avoidance/PathControlAction.h>

#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <collision_avoidance/obstacle_restriction_method.h>
#include <collision_avoidance/polar_histogram.h>

#include <collision_avoidance/CollisionAvoidanceConfig.h>
#include <dynamic_reconfigure/server.h>

namespace collision_avoidance
{
class CollisionAvoidance
{
private:
  // Subscribers
  std::vector<ros::Subscriber> cloud_sub_;
  ros::Subscriber sensor_sub_;
  ros::Subscriber odometry_sub_;

  // Publishers
  ros::Publisher control_pub_;
  ros::Publisher path_pub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher current_setpoint_;

  // Action servers
  actionlib::SimpleActionServer<collision_avoidance::PathControlAction> as_;

  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<collision_avoidance::CollisionAvoidanceConfig> cs_;
  dynamic_reconfigure::Server<collision_avoidance::CollisionAvoidanceConfig>::CallbackType f_;

  // Stored data
  std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds_;
  nav_msgs::Odometry::ConstPtr odometry_;

  // Current target
  geometry_msgs::PoseStamped target_;

  // ORM
  ORM orm_;

  // Robot frame_id
  std::string robot_frame_id_;

  // Converge criterion
  double distance_converged_;
  double yaw_converged_;

  // Publish frequency
  double frequency_;

  // Timers
  ros::Timer publish_timer_;

  // Max allowed velocity
  double max_xy_vel_;
  double max_z_vel_;
  double max_yaw_rate_;
  double h_m_;

  //
  double max_direction_change_;
  int max_times_backwards_;

  int num_histogram_;

  double radius_;
  double height_;
  double min_distance_hold_;

  double leaf_size_;

  double look_ahead_distance_;
  bool look_forward_;
  bool move_while_yawing_;
  bool yaw_each_setpoint_;

public:
  CollisionAvoidance(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
  void goalCallback(const collision_avoidance::PathControlGoal::ConstPtr& goal);

  geometry_msgs::PoseStamped getNextSetpoint(nav_msgs::Path* path) const;

  geometry_msgs::Pose interpolate(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end, double t) const;

  geometry_msgs::Point lerp(const geometry_msgs::Point& start, const geometry_msgs::Point& end, double t) const;

  geometry_msgs::Quaternion slerp(const geometry_msgs::Quaternion& start, const geometry_msgs::Quaternion& end,
                                  double t) const;

  bool avoidCollision(geometry_msgs::PoseStamped setpoint);

  void noInput(geometry_msgs::PoseStamped setpoint) const;

  void adjustVelocity(geometry_msgs::TwistStamped* control, const PolarHistogram& obstacles) const;

  PolarHistogram getObstacles(double obstacle_window, double height_diff = 0.0) const;

  std::pair<double, double> getDistanceToTarget(const geometry_msgs::PoseStamped& target);

  void timerCallback(const ros::TimerEvent& event);

  void publishObstacles(const PolarHistogram& obstacles) const;

  void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, int index)
  {
    clouds_[index] = cloud;
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry)
  {
    odometry_ = odometry;
  }

  void configCallback(const collision_avoidance::CollisionAvoidanceConfig& config, uint32_t level);
};
}  // namespace collision_avoidance

#endif  // COLLISION_AVOIDANCE_H