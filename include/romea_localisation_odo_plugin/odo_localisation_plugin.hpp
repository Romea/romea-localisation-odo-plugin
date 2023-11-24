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

#ifndef ROMEA_LOCALISATION_ODO_PLUGIN__ODO_LOCALISATION_PLUGIN_HPP_
#define ROMEA_LOCALISATION_ODO_PLUGIN__ODO_LOCALISATION_PLUGIN_HPP_

// Eigen
#include <Eigen/Core>

// ros
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

// romea
#include "romea_localisation_odo_plugin/visibility_control.h"
#include "romea_mobile_base_msgs/msg/kinematic_measure_stamped.hpp"
#include "romea_localisation_msgs/msg/observation_twist2_d_stamped.hpp"

namespace romea
{
namespace ros2
{

class OdoLocalisationPlugin
{
public:
  using OdometryMsg = nav_msgs::msg::Odometry;
  using KinematicMeasureStampedMsg = romea_mobile_base_msgs::msg::KinematicMeasureStamped;
  using ObservationTwist2DStampedMsg = romea_localisation_msgs::msg::ObservationTwist2DStamped;

public:
  ROMEA_LOCALISATION_ODO_PLUGIN_PUBLIC
  explicit OdoLocalisationPlugin(const rclcpp::NodeOptions & options);

  ROMEA_LOCALISATION_ODO_PLUGIN_PUBLIC
  virtual ~OdoLocalisationPlugin() = default;

  ROMEA_LOCALISATION_ODO_PLUGIN_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private:
  void declare_parameters_();

  void process_odom_(OdometryMsg::ConstSharedPtr msg);

  void process_kinematic_(KinematicMeasureStampedMsg::ConstSharedPtr msg);

  void init_subscriber_();

  void init_odom_subscriber_();

  void init_kinematic_subscriber_();

  void init_publisher_();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<OdometryMsg>::SharedPtr odom_sub_;
  rclcpp::Subscription<KinematicMeasureStampedMsg>::SharedPtr kinematic_sub_;
  rclcpp::Publisher<ObservationTwist2DStampedMsg>::SharedPtr twist_pub_;
  bool restamping_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_LOCALISATION_ODO_PLUGIN__ODO_LOCALISATION_PLUGIN_HPP_
