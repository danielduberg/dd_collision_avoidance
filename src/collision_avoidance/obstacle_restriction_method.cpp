#include <collision_avoidance/no_input.h>
#include <collision_avoidance/obstacle_restriction_method.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

namespace collision_avoidance
{
int mod(int a, int b)
{
  return ((a %= b) < 0) ? a + b : a;
}

ORM::ORM(double radius, double security_distance, double epsilon)
  : radius_(radius), security_distance_(security_distance), epsilon_(epsilon)
{
}

ORM::ORM(ros::NodeHandle& nh, double radius, double security_distance, double epsilon)
  : radius_(radius)
  , security_distance_(security_distance)
  , epsilon_(epsilon)
  , goal_pub_(nh.advertise<geometry_msgs::PointStamped>("goal", 10))
  , subgoal_pub_(nh.advertise<geometry_msgs::PointStamped>("subgoal", 10))
  , obstacles_pub_(nh.advertise<visualization_msgs::MarkerArray>("points_of_interest", 10))
{
}

Eigen::Vector2d ORM::avoidCollision(const Eigen::Vector2d& goal, const PolarHistogram& obstacles,
                                    const std::string& frame_id)
{
  Eigen::Vector2d subgoal(subgoalSelector(goal, obstacles));

  if ("" != frame_id && subgoal_pub_.getNumSubscribers() > 0)
  {
    geometry_msgs::PointStamped subgoal_point;
    subgoal_point.header.frame_id = frame_id;
    subgoal_point.header.stamp = ros::Time::now();
    subgoal_point.point.x = subgoal[0];
    subgoal_point.point.y = subgoal[1];
    subgoal_point.point.z = 0;
    subgoal_pub_.publish(subgoal_point);
  }

  if (subgoal.isZero())
  {
    return subgoal;
  }

  Eigen::Vector2d final_goal(motionComputation(subgoal, obstacles));

  if ("" != frame_id && goal_pub_.getNumSubscribers() > 0)
  {
    geometry_msgs::PointStamped goal_point;
    goal_point.header.frame_id = frame_id;
    goal_point.header.stamp = ros::Time::now();
    goal_point.point.x = final_goal[0];
    goal_point.point.y = final_goal[1];
    goal_point.point.z = 0;
    goal_pub_.publish(goal_point);
  }

  return final_goal;
}

Eigen::Vector2d ORM::subgoalSelector(const Eigen::Vector2d& goal, const PolarHistogram& obstacles)
{
  if (isPathClear(goal, obstacles))
  {
    // We can go straight towards goal
    return goal;
  }

  // We have to find a subgoal
  double goal_direction = std::atan2(goal[1], goal[0]);

  Eigen::Vector2d subgoal(goal);
  for (double d = obstacles.bucketSize(); d < M_PI; d += obstacles.bucketSize())
  {
    for (double i : { -1, 1 })
    {
      double current_direction = goal_direction + (i * d);
      double previous_direction = goal_direction + (i * (d - obstacles.bucketSize()));
      if (isSubgoal(obstacles, current_direction, previous_direction, &subgoal))
      {
        return subgoal;
      }
    }
  }

  return Eigen::Vector2d(0, 0);
}

bool ORM::isSubgoal(const PolarHistogram& obstacles, double direction_1, double direction_2, Eigen::Vector2d* subgoal)
{
  // TODO: Maybe check for NaN? Or assume that it is never NaN?
  if (!obstacles.isFinite(direction_1) && !obstacles.isFinite(direction_2))
  {
    // Not a subgoal becuase it is only open space
    return false;
  }

  if (!obstacles.isFinite(direction_1))
  {
    // This makes it so direction_1 is always finite
    double temp = direction_1;
    direction_1 = direction_2;
    direction_2 = temp;
  }

  Eigen::Vector2d temp_subgoal;

  if (!obstacles.isFinite(direction_2))
  {
    // Found a potential subgoal at the edge of an obstacle
    temp_subgoal[0] = obstacles.getRange(direction_1) * std::cos(direction_2);
    temp_subgoal[1] = obstacles.getRange(direction_1) * std::sin(direction_2);
  }
  else
  {
    // We know that both are finite now
    // TODO: Maybe save the points?
    if ((obstacles.getPoint(direction_2) - obstacles.getPoint(direction_1)).norm() > 2.0 * radius_)
    {
      // Found a potential subgoal between two obstacles
      temp_subgoal = (obstacles.getPoint(direction_1) + obstacles.getPoint(direction_2)) / 2.0;
    }
    else
    {
      return false;
    }
  }

  if (isPathClear(temp_subgoal, obstacles))
  {
    *subgoal = temp_subgoal;
    return true;
  }
  return false;
}

Eigen::Vector2d ORM::motionComputation(const Eigen::Vector2d& goal, const PolarHistogram& obstacles)
{
  double goal_direction = std::atan2(goal[1], goal[0]);

  std::pair<double, double> bounds = getBounds(goal, obstacles);

  double diff_left = radBounded(bounds.first - goal_direction);
  double diff_right = radBounded(bounds.second - goal_direction);

  if (diff_left < 0 && diff_right > 0)
  {
    // Move Straight towards goal
    return goal;
  }

  double direction;
  if (diff_right > 0 && diff_right > diff_left)
  {
    // Move towards left bound
    direction = bounds.first;
  }
  else if (diff_left < 0 && diff_right > diff_left)
  {
    // Move towards right bound
    direction = bounds.second;
  }
  else
  {
    // Move to the middle of left and right bound
    Eigen::Vector2d left_bound(std::cos(bounds.first), std::sin(bounds.first));
    Eigen::Vector2d right_bound(std::cos(bounds.second), std::sin(bounds.second));
    Eigen::Vector2d middle = (left_bound + right_bound) / 2.0;
    direction = std::atan2(middle[1], middle[0]);
  }

  return goal.norm() * Eigen::Vector2d(std::cos(direction), std::sin(direction));
}

std::pair<double, double> ORM::getBounds(const Eigen::Vector2d& goal, const PolarHistogram& obstacles)
{
  // TODO: Should we only care about the obstacles that are in the way?

  double max_distance = goal.norm() + radius_;  // TODO: + epsilon_ also?

  double goal_direction = std::atan2(goal[1], goal[0]);

  double left_bound = radBounded(goal_direction - (M_PI - 0.001));
  double right_bound = radBounded(goal_direction + (M_PI - 0.001));

  for (const PolarHistogram::Vector& obstacle : obstacles)
  {
    if (!obstacle.isFinite() || obstacle.getRange() > max_distance)
    {
      continue;
    }

    // TODO: atan or atan2?
    // double alpha = std::fabs(std::atan((radius_ + security_distance_) / obstacle.getRange()));
    double alpha = std::fabs(std::atan2(radius_ + security_distance_, obstacle.getRange()));

    double beta = 0;
    if (obstacle.getRange() < radius_ + security_distance_)
    {
      beta = (M_PI - alpha) * (1.0 - ((obstacle.getRange() - radius_) / security_distance_));
    }

    double alpha_beta = alpha + beta;

    double obstacle_direction_goal_origin = radBounded(obstacle.getAngle() - goal_direction);
    if (obstacle_direction_goal_origin > 0)
    {
      if (obstacle_direction_goal_origin - alpha_beta < radBounded(right_bound - goal_direction))
      {
        right_bound = obstacle.getAngle() - alpha_beta;
      }
    }
    else
    {
      if (obstacle_direction_goal_origin + alpha_beta > radBounded(left_bound - goal_direction))
      {
        left_bound = obstacle.getAngle() + alpha_beta;
      }
    }
  }

  return std::make_pair(left_bound, right_bound);
}

bool ORM::isPathClear(const Eigen::Vector2d& goal, const PolarHistogram& obstacles)
{
  // A contains points on the left side of goal
  // B contains points on the right side of goal
  std::vector<Eigen::Vector2d> A, B;

  getPointsOfInterest(goal, obstacles, &A, &B);

  // Corners of the rectangle (tunnel)
  std::vector<Eigen::Vector2d> rectangle = getRectangle(goal);

  A = getPointsInPolygon(rectangle, A);
  B = getPointsInPolygon(rectangle, B);

  // Check if path is clear
  double squared_diameter = std::pow(2.0 * radius_, 2.0);
  for (const Eigen::Vector2d& a : A)
  {
    for (const Eigen::Vector2d& b : B)
    {
      if ((b - a).squaredNorm() < squared_diameter)
      {
        return false;
      }
    }
  }

  return true;
}

void ORM::getPointsOfInterest(const Eigen::Vector2d& goal, const PolarHistogram& obstacles,
                              std::vector<Eigen::Vector2d>* left, std::vector<Eigen::Vector2d>* right)
{
  double goal_direction = std::atan2(goal[1], goal[0]);
  double max_distance = goal.norm() + radius_;

  for (double d = 0; d < M_PI; d += obstacles.bucketSize())
  {
    double direction = goal_direction + d;
    if (obstacles.isFinite(direction) && obstacles.getRange(direction) <= max_distance)
    {
      left->emplace_back(obstacles.getPoint(direction));
    }

    direction = goal_direction - d;
    if (obstacles.isFinite(direction) && obstacles.getRange(direction) <= max_distance)
    {
      right->emplace_back(obstacles.getPoint(direction));
    }
  }
}

std::vector<Eigen::Vector2d> ORM::getPointsInPolygon(const std::vector<Eigen::Vector2d>& polygon,
                                                     const std::vector<Eigen::Vector2d> points)
{
  std::vector<Eigen::Vector2d> points_in_polygon;

  for (const Eigen::Vector2d& point : points)
  {
    if (!point.allFinite())
    {
      // Not possible to be inside the polygon
      continue;
    }

    if (isPointInPolygon(polygon, point))
    {
      points_in_polygon.emplace_back(point);
    }
  }

  return points_in_polygon;
}

// Source:
// https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
bool ORM::isPointInPolygon(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& point)
{
  bool c = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
  {
    if (((polygon[i][1] > point[1]) != (polygon[j][1] > point[1])) &&
        (point[0] < (polygon[j][0] - polygon[i][0]) * (point[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) +
                        polygon[i][0]))
    {
      c = !c;
    }
  }

  return c;
}

std::vector<Eigen::Vector2d> ORM::getRectangle(const Eigen::Vector2d& goal)
{
  double distance = goal.norm() + radius_;
  double direction = std::atan2(goal[1], goal[0]);

  Eigen::Vector2d moved_goal(distance * std::cos(direction), distance * std::sin(direction));

  double dx = moved_goal[1] / distance;
  double dy = -moved_goal[0] / distance;

  // 2 * radius becuase then all obstacles that matters will be inside the rectangle
  double diameter = 2.0 * radius_;

  std::vector<Eigen::Vector2d> rectangle;
  rectangle.emplace_back(diameter * dx, diameter * dy);
  rectangle.emplace_back(moved_goal[0] + rectangle[0][0], moved_goal[1] + rectangle[0][1]);
  rectangle.emplace_back(moved_goal[0] - rectangle[0][0], moved_goal[1] - rectangle[0][1]);
  rectangle.emplace_back(-rectangle[0][0], -rectangle[0][1]);

  return rectangle;
}
}  // namespace collision_avoidance
