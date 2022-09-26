#include "romea_localisation_odo/odo_localisation_plugin.hpp"
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_common_utils/qos.hpp>
#include <rclcpp/logging.hpp>

namespace{
const std::string restamping_param_name = "restamping";
const std::string odo_source_param_name = "odo_source";
}

namespace romea{


//-----------------------------------------------------------------------------
OdoLocalisationPlugin::OdoLocalisationPlugin(const rclcpp::NodeOptions & options):
  node_(std::make_shared<rclcpp::Node>("odo_localisation_plugin",options)),
  odom_sub_(nullptr),
  kinematic_sub_(nullptr),
  twist_pub_(),
  restamping_(false)
{

  declare_parameters_();
  restamping_ = get_parameter<bool>(node_,restamping_param_name);

  init_publisher_();
  init_subscriber_();
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
OdoLocalisationPlugin::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
void OdoLocalisationPlugin::declare_parameters_()
{
  declare_parameter_with_default<bool>(node_,restamping_param_name,false);
  declare_parameter<std::string>(node_,odo_source_param_name,"kinematic");
}

//-----------------------------------------------------------------------------
void OdoLocalisationPlugin::init_subscriber_()
{
  std::string odo_source_name = get_parameter<std::string>(node_,odo_source_param_name);

  if(odo_source_name=="odom")
  {
    init_odom_subscriber_();
  }
  else if (odo_source_name=="kinematic")
  {
    init_kinematic_subscriber_();
  }
  else
  {
    throw  std::runtime_error("Unknown odo_source called : " + odo_source_name);
  }
}

//-----------------------------------------------------------------------------
void OdoLocalisationPlugin::init_odom_subscriber_()
{
  auto callback = std::bind(&OdoLocalisationPlugin::process_odom_,this,std::placeholders::_1);
  odom_sub_ = node_->create_subscription<OdometryMsg>("vehicle_controller/odom",best_effort(1),callback);
}

//-----------------------------------------------------------------------------
void OdoLocalisationPlugin::init_kinematic_subscriber_()
{
  auto callback = std::bind(&OdoLocalisationPlugin::process_kinematic_,this,std::placeholders::_1);
  kinematic_sub_ = node_->create_subscription<KinematicMeasureStampedMsg>("vehicle_controller/kinematic",best_effort(1),callback);
}

//-----------------------------------------------------------------------------
void OdoLocalisationPlugin::init_publisher_()
{
  twist_pub_ = node_->create_publisher<ObservationTwist2DStampedMsg>("twist",sensor_data_qos());
}

//-----------------------------------------------------------------------------
void OdoLocalisationPlugin::process_odom_(OdometryMsg::ConstSharedPtr msg)
{

  //std::cout << " processOdom_ "<< std::endl;

  auto twist_msg = std::make_unique<romea_localisation_msgs::msg::ObservationTwist2DStamped>();

  twist_msg->header.frame_id = msg->header.frame_id;
  twist_msg->header.stamp = restamping_ ? node_->get_clock()->now() : rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
  twist_msg->observation_twist.twist.linear_speeds.x =msg->twist.twist.linear.x;
  twist_msg->observation_twist.twist.linear_speeds.y =msg->twist.twist.linear.y;
  twist_msg->observation_twist.twist.angular_speed =msg->twist.twist.angular.z;

  //  twist_msg->twist.covariance[0] = msg->twist.covariance[0];
  //  twist_msg->twist.covariance[1] = msg->twist.covariance[1];
  //  twist_msg->twist.covariance[2] = msg->twist.covariance[5];
  //  twist_msg->twist.covariance[3] = msg->twist.covariance[6];
  //  twist_msg->twist.covariance[4] = msg->twist.covariance[7];
  //  twist_msg->twist.covariance[5] = msg->twist.covariance[11];
  //  twist_msg->twist.covariance[6] = msg->twist.covariance[30];
  //  twist_msg->twist.covariance[7] = msg->twist.covariance[31];
  //  twist_msg->twist.covariance[8] = msg->twist.covariance[35];

  twist_msg->observation_twist.twist.covariance[0] = 0.01;
  twist_msg->observation_twist.twist.covariance[1] = 0;
  twist_msg->observation_twist.twist.covariance[2] = 0;
  twist_msg->observation_twist.twist.covariance[3] = 0;
  twist_msg->observation_twist.twist.covariance[4] = 0.01;
  twist_msg->observation_twist.twist.covariance[5] = 0;
  twist_msg->observation_twist.twist.covariance[6] = 0;
  twist_msg->observation_twist.twist.covariance[7] = 0;
  twist_msg->observation_twist.twist.covariance[8] = 0.002;
  twist_pub_->publish(std::move(twist_msg));
}

//-----------------------------------------------------------------------------
void OdoLocalisationPlugin::process_kinematic_(KinematicMeasureStampedMsg::ConstSharedPtr msg)
{

  //std::cout << " processKinematic "<< std::endl;

  auto twist_msg = std::make_unique<romea_localisation_msgs::msg::ObservationTwist2DStamped>();

  twist_msg->header.frame_id = msg->header.frame_id;
  twist_msg->header.stamp = restamping_ ? node_->get_clock()->now() : rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
  twist_msg->observation_twist.twist.linear_speeds.x =msg->measure.longitudinal_speed;
  twist_msg->observation_twist.twist.linear_speeds.y =msg->measure.lateral_speed;
  twist_msg->observation_twist.twist.angular_speed =msg->measure.angular_speed;

  //    twist_msg->twist.covariance[0] = msg->measure.covariance[0];
  //    twist_msg->twist.covariance[1] = msg->measure.covariance[1];
  //    twist_msg->twist.covariance[2] = msg->measure.covariance[2];
  //    twist_msg->twist.covariance[3] = msg->measure.covariance[4];
  //    twist_msg->twist.covariance[4] = msg->measure.covariance[5];
  //    twist_msg->twist.covariance[5] = msg->measure.covariance[6];
  //    twist_msg->twist.covariance[6] = msg->measure.covariance[8];
  //    twist_msg->twist.covariance[7] = msg->measure.covariance[9];
  //    twist_msg->twist.covariance[8] = msg->measure.covariance[10];

  twist_msg->observation_twist.twist.covariance[0] = 0.01;
  twist_msg->observation_twist.twist.covariance[1] = 0;
  twist_msg->observation_twist.twist.covariance[2] = 0;
  twist_msg->observation_twist.twist.covariance[3] = 0;
  twist_msg->observation_twist.twist.covariance[4] = 0.01;
  twist_msg->observation_twist.twist.covariance[5] = 0;
  twist_msg->observation_twist.twist.covariance[6] = 0;
  twist_msg->observation_twist.twist.covariance[7] = 0;
  twist_msg->observation_twist.twist.covariance[8] = 0.002;
  twist_pub_->publish(std::move(twist_msg));

}

}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(romea::OdoLocalisationPlugin)

