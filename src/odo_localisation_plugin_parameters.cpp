#include "romea_localisation_odo/odo_localisation_plugin_parameters.hpp"
#include <romea_common_utils/params/node_parameters.hpp>

namespace  {
const char restamping_param_name[] = "restamping";
const char odo_source_param_name[] = "odo_source";
}

namespace romea
{

//-----------------------------------------------------------------------------
void declare_restamping(rclcpp::Node::SharedPtr node)
{
  declare_parameter_with_default<bool>(node, restamping_param_name, false);
}

//-----------------------------------------------------------------------------
void declare_odo_source(rclcpp::Node::SharedPtr node)
{
  declare_parameter<std::string>(node, odo_source_param_name, "kinematic");
}

//-----------------------------------------------------------------------------
bool get_restamping(rclcpp::Node::SharedPtr node)
{
  return get_parameter<bool>(node, restamping_param_name);
}

//-----------------------------------------------------------------------------
std::string get_odo_source(rclcpp::Node::SharedPtr node)
{
  return get_parameter<std::string>(node, odo_source_param_name);
}

}  // namespace romea
