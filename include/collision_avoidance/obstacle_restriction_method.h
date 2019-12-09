#ifndef COLLISION_AVOIDANCE_OBSTACLE_RESTRICTION_METHOD_H
#define COLLISION_AVOIDANCE_OBSTACLE_RESTRICTION_METHOD_H

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>

#include <collision_avoidance/polar_histogram.h>

// TODO: Make methods const

namespace collision_avoidance
{
// Obstacle-Restriction Method
class ORM
{
private:
  double radius_;
  double security_distance_;
  double epsilon_;

  ros::Publisher goal_pub_;
  ros::Publisher subgoal_pub_;
  ros::Publisher obstacles_pub_;

public:
  ORM(double radius = 0.0, double security_distance = 0.0, double epsilon = 0.0);

  ORM(ros::NodeHandle& nh, double radius = 0.0, double security_distance = 0.0, double epsilon = 0.0);

  Eigen::Vector2d avoidCollision(const Eigen::Vector2d& goal, const PolarHistogram& obstacles,
                                 const std::string& frame_id = "");

  double getRadius() const
  {
    return radius_;
  }
  double getSecurityDistance() const
  {
    return security_distance_;
  }
  double getEpsilon() const
  {
    return epsilon_;
  }

  void setRadius(double radius)
  {
    radius_ = radius;
  }

  void setSecurityDistance(double security_distance)
  {
    security_distance_ = security_distance;
  }

  void setEpsilon(double epsilon)
  {
    epsilon_ = epsilon;
  }

private:
  inline double radBounded(double value)
  {
    return std::remainder(value, 2.0 * M_PI);
  }

  Eigen::Vector2d subgoalSelector(const Eigen::Vector2d& goal, const PolarHistogram& obstacles);

  bool isSubgoal(const PolarHistogram& obstacles, double direction_1, double direction_2, Eigen::Vector2d* subgoal);

  Eigen::Vector2d motionComputation(const Eigen::Vector2d& goal, const PolarHistogram& obstacles);

  std::pair<double, double> getBounds(const Eigen::Vector2d& goal, const PolarHistogram& obstacles);

  bool isPathClear(const Eigen::Vector2d& goal, const PolarHistogram& obstacles);

  void getPointsOfInterest(const Eigen::Vector2d& goal, const PolarHistogram& obstacles,
                           std::vector<Eigen::Vector2d>* left, std::vector<Eigen::Vector2d>* right);

  std::vector<Eigen::Vector2d> getRectangle(const Eigen::Vector2d& goal);

  std::vector<Eigen::Vector2d> getPointsInPolygon(const std::vector<Eigen::Vector2d>& polygon,
                                                  const std::vector<Eigen::Vector2d> points);

  bool isPointInPolygon(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& point);
};
}  // namespace collision_avoidance

#endif  // COLLISION_AVOIDANCE_OBSTACLE_RESTRICTION_METHOD_H