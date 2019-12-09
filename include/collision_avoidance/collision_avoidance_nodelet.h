#ifndef COLLISION_AVOIDANCE_NODELET_H
#define COLLISION_AVOIDANCE_NODELET_H

#include <nodelet/nodelet.h>

#include <collision_avoidance/collision_avoidance.h>

namespace collision_avoidance
{
class CollisionAvoidanceNodelet : public nodelet::Nodelet
{
private:
  std::shared_ptr<CollisionAvoidance> collision_avoidance_;

public:
  virtual void onInit();
};
}  // namespace collision_avoidance

#endif  // COLLISION_AVOIDANCE_NODELET_H