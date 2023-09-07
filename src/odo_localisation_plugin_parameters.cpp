// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <string>

// romea
#include "romea_localisation_odo_plugin/odo_localisation_plugin_parameters.hpp"
#include "romea_common_utils/params/node_parameters.hpp"

namespace
{
const char restamping_param_name[] = "restamping";
const char controller_topic_param_name[] = "controller_type";
}

namespace romea
{

//-----------------------------------------------------------------------------
void declare_restamping(rclcpp::Node::SharedPtr node)
{
  declare_parameter_with_default<bool>(node, restamping_param_name, false);
}

//-----------------------------------------------------------------------------
void declare_controller_topic(rclcpp::Node::SharedPtr node)
{
  declare_parameter_with_default<std::string>(
    node, controller_topic_param_name, "kinematic");
}

//-----------------------------------------------------------------------------
bool get_restamping(rclcpp::Node::SharedPtr node)
{
  return get_parameter<bool>(node, restamping_param_name);
}

//-----------------------------------------------------------------------------
std::string get_controller_topic(rclcpp::Node::SharedPtr node)
{
  return get_parameter<std::string>(node, controller_topic_param_name);
}

}  // namespace romea
