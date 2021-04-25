#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct requestInfo
{
  geometry_msgs::msg::PoseStamped goal;
  std::string planner_id;
  std::string controller_id;
};