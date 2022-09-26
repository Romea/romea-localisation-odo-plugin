//ros
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

//romea
#include "visibility_control.h"
#include <romea_mobile_base_msgs/msg/kinematic_measure_stamped.hpp>
#include <romea_localisation_msgs/msg/observation_twist2_d_stamped.hpp>

//Eigen
#include <Eigen/Core>

namespace romea {


class OdoLocalisationPlugin
{
public:

  using OdometryMsg = nav_msgs::msg::Odometry;
  using KinematicMeasureStampedMsg = romea_mobile_base_msgs::msg::KinematicMeasureStamped;
  using ObservationTwist2DStampedMsg = romea_localisation_msgs::msg::ObservationTwist2DStamped;

public:

  ROMEA_LOCALISATION_ODO_PUBLIC
  OdoLocalisationPlugin(const rclcpp::NodeOptions & options);

  ROMEA_LOCALISATION_ODO_PUBLIC
  virtual ~OdoLocalisationPlugin()=default;

  ROMEA_LOCALISATION_ODO_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private :

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

}
