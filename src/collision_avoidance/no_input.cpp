#include <collision_avoidance/no_input.h>

namespace collision_avoidance
{
namespace no_input
{
Eigen::Vector2d avoidCollision(const Eigen::Vector2d& goal, const PolarHistogram& obstacles, double radius,
                               double min_distance_hold)
{
  double x_min = 1000000;
  double y_min = 1000000;
  double x_max = -1000000;
  double y_max = -1000000;

  double closest_distance = radius + min_distance_hold;
  for (const PolarHistogram::Vector& obstacle : obstacles)
  {
    if (!obstacle.isFinite())
    {
      continue;
    }

    if (obstacle.getRange() < closest_distance)
    {
      double magnitude = closest_distance - obstacle.getRange();
      double direction = obstacle.getAngle() + M_PI;  // We want to move in the opposite direction

      Eigen::Vector2d p(magnitude * std::cos(direction), magnitude * std::sin(direction));

      x_min = std::min(x_min, p[0]);
      x_max = std::max(x_max, p[0]);
      y_min = std::min(y_min, p[1]);
      y_max = std::max(y_max, p[1]);
    }
  }

  return goal + Eigen::Vector2d((x_min + x_max) / 2.0, (y_min + y_max) / 2.0);
}
}  // namespace no_input
}  // namespace collision_avoidance
