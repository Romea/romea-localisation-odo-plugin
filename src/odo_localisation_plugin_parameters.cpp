// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// std
#include <string>

// romea
#include "romea_localisation_odo_plugin/odo_localisation_plugin_parameters.hpp"
#include "romea_common_utils/params/node_parameters.hpp"

namespace
{
const char restamping_param_name[] = "restamping";
const char controller_topic_param_name[] = "controller_topic";
}

namespace romea
{
namespace ros2
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

}  // namespace ros2
}  // namespace romea
