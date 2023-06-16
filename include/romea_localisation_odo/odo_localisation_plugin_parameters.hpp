// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_LOCALISATION_ODO__ODO_LOCALISATION_PLUGIN_PARAMETERS_HPP_
#define ROMEA_LOCALISATION_ODO__ODO_LOCALISATION_PLUGIN_PARAMETERS_HPP_

// std
#include <string>

// ros
#include "rclcpp/node.hpp"

namespace romea
{

void declare_restamping(rclcpp::Node::SharedPtr node);

void declare_controller_topic(rclcpp::Node::SharedPtr node);


bool get_restamping(rclcpp::Node::SharedPtr node);

std::string get_controller_topic(rclcpp::Node::SharedPtr node);

}  // namespace romea

#endif  // ROMEA_LOCALISATION_ODO__ODO_LOCALISATION_PLUGIN_PARAMETERS_HPP_
