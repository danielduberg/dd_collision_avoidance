#include <collision_avoidance/collision_avoidance_nodelet.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(collision_avoidance::CollisionAvoidanceNodelet, nodelet::Nodelet)

namespace collision_avoidance
{
void CollisionAvoidanceNodelet::onInit()
{
  ros::NodeHandle& nh = getMTNodeHandle();  // getNodeHandle();
  ros::NodeHandle& nh_priv = getMTPrivateNodeHandle();
  collision_avoidance_ = std::make_shared<CollisionAvoidance>(nh, nh_priv);
}
}  // namespace collision_avoidance