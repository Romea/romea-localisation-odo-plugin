#include "romea_localisation_odo/odo_localisation_plugin.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::executors::SingleThreadedExecutor exec;
  romea::OdoLocalisationPlugin plugin(options);
  exec.add_node(plugin.get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
