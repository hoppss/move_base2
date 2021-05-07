#include <memory>

#include "move_base2/MoveBase.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<move_base::MoveBase>();

  node->loop();
  // rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
