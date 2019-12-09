#ifndef COLLISION_AVOIDANCE_HISTOGRAM_H
#define COLLISION_AVOIDANCE_HISTOGRAM_H

#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace collision_avoidance
{
class PolarHistogram
{
public:
  struct Vector
  {
  private:
    double angle_;
    double range_;

  public:
    Vector(double angle, double range = 0.0) : angle_(angle), range_(range)
    {
    }

    double getRange() const
    {
      return range_;
    }

    double getAngle() const
    {
      return angle_;
    }

    Eigen::Vector2d getPoint() const
    {
      return Eigen::Vector2d(range_ * std::cos(angle_), range_ * std::sin(angle_));
    }

    double setRange(double range)
    {
      range_ = range;
    }

    bool isFinite() const
    {
      return std::isfinite(range_);
    }
  };

private:
  std::vector<Vector> histogram_;

  double bucket_size_;
  double half_bucket_size_;

private:
  inline double radBounded(double value) const
  {
    return std::remainder(value, 2.0 * M_PI);
  }

  inline double posRad(double value) const
  {
    value = radBounded(value);
    if (value < 0)
    {
      return value + (2.0 * M_PI);
    }
    return value;
  }

public:
  PolarHistogram(size_t num_buckets, double init_range)
    : bucket_size_(2.0 * M_PI / num_buckets), half_bucket_size_(bucket_size_ / 2.0)
  {
    histogram_.reserve(num_buckets);
    for (double angle = 0; angle < 2.0 * M_PI; angle += bucket_size_)
    {
      histogram_.emplace_back(angle, init_range);
    }
  }

  inline double bucketSize() const
  {
    return bucket_size_;
  }

  inline size_t numBuckets() const
  {
    return histogram_.size();
  }

  Vector& get(double direction)
  {
    direction += half_bucket_size_;
    return histogram_[posRad(direction) / bucket_size_];
  }

  Vector& get(double x, double y)
  {
    return get(std::atan2(y, x));
  }

  Vector& get(const Eigen::Vector2d& point)
  {
    return get(point[0], point[1]);
  }

  const Vector& get(double direction) const
  {
    direction += half_bucket_size_;
    return histogram_[posRad(direction) / bucket_size_];
  }

  const Vector& get(double x, double y) const
  {
    return get(std::atan2(y, x));
  }

  const Vector& get(const Eigen::Vector2d& point) const
  {
    return get(point[0], point[1]);
  }

  double getRange(double direction) const
  {
    direction += half_bucket_size_;
    return histogram_[posRad(direction) / bucket_size_].getRange();
  }

  double getRange(double x, double y) const
  {
    return getRange(std::atan2(y, x));
  }

  double getRange(const Eigen::Vector2d& point) const
  {
    return getRange(point[0], point[1]);
  }

  void setRange(double direction, double range)
  {
    direction += half_bucket_size_;
    histogram_[posRad(direction) / bucket_size_].setRange(range);
  }

  Eigen::Vector2d getPoint(double direction) const
  {
    direction += half_bucket_size_;
    return histogram_[posRad(direction) / bucket_size_].getPoint();
  }

  Eigen::Vector2d getPoint(double x, double y) const
  {
    return getPoint(std::atan2(y, x));
  }

  Eigen::Vector2d getPoint(const Eigen::Vector2d& point) const
  {
    return getPoint(point[0], point[1]);
  }

  void insertPoint(double x, double y)
  {
    setRange(std::atan2(y, x), std::hypot(x, y));
  }

  void insertPoint(const Eigen::Vector2d& point)
  {
    setRange(std::atan2(point[1], point[0]), point.norm());
  }

  bool isFinite(double direction) const
  {
    return get(direction).isFinite();
  }

  bool isFinite(double x, double y) const
  {
    return isFinite(std::atan2(y, x), std::hypot(x, y));
  }

  bool isFinite(const Eigen::Vector2d& point)
  {
    return isFinite(std::atan2(point[1], point[0]), point.norm());
  }

  Vector& operator[](double direction)
  {
    return get(direction);
  }

  const Vector& operator[](double direction) const
  {
    return get(direction);
  }

  std::vector<Vector>::iterator begin()
  {
    return histogram_.begin();
  }

  std::vector<Vector>::const_iterator begin() const
  {
    return histogram_.begin();
  }

  std::vector<Vector>::iterator end()
  {
    return histogram_.end();
  }

  std::vector<Vector>::const_iterator end() const
  {
    return histogram_.end();
  }
};
}  // namespace collision_avoidance

#endif  // COLLISION_AVOIDANCE_HISTOGRAM_H