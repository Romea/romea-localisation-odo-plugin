#ifndef __OdoLocalisationPluginParameters_HPP__
#define __OdoLocalisationPluginParameters_HPP__

#include <rclcpp/node.hpp>

namespace romea {

void declare_restamping(rclcpp::Node::SharedPtr node);

void declare_odo_source(rclcpp::Node::SharedPtr node);


bool get_restamping(rclcpp::Node::SharedPtr node);

std::string get_odo_source(rclcpp::Node::SharedPtr node);

}

#endif
