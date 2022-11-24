#ifndef ROMEA_LOCALISATION_ODO_ODO_LOCALISATION_PLUGIN_PARAMETERS_HPP_
#define ROMEA_LOCALISATION_ODO_ODO_LOCALISATION_PLUGIN_PARAMETERS_HPP_

// std
#include <string>

// ro
#include <rclcpp/node.hpp>

namespace romea {

void declare_restamping(rclcpp::Node::SharedPtr node);

void declare_odo_source(rclcpp::Node::SharedPtr node);


bool get_restamping(rclcpp::Node::SharedPtr node);

std::string get_odo_source(rclcpp::Node::SharedPtr node);

}  // namespace romea

#endif  // ROMEA_LOCALISATION_ODO_ODO_LOCALISATION_PLUGIN_PARAMETERS_HPP_
