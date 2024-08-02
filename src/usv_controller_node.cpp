#include <memory>
#include <usv_controller/usv_controller_component.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<usv_controller::UsvControllerComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}