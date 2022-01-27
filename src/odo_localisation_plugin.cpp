#include "odo_localisation_plugin.hpp"
#include <romea_common_utils/params/node_parameters.hpp>
#include <rclcpp/logging.hpp>

namespace romea{


//-----------------------------------------------------------------------------
OdoLocalisationPlugin::OdoLocalisationPlugin():
  Node("odo_localisation_plugin"),
  odom_sub_(nullptr),
  kinematic_sub_(nullptr),
  twist_pub_(),
  restamping_(false)
{
}

//-----------------------------------------------------------------------------
void OdoLocalisationPlugin::onInit()
{

  auto node_parameters = NodeParameters(this->shared_from_this());

  restamping_ = node_parameters.loadParamOr<bool>("restamping",false);
  std::string kinematicSourceName = node_parameters.loadParamOr<std::string>("odo_source","kinematic");
  twist_pub_ = create_publisher<romea_localisation_msgs::msg::ObservationTwist2DStamped>("twist",1);

  if(kinematicSourceName=="odom")
  {
    auto callback = std::bind(&OdoLocalisationPlugin::processOdom_,this,std::placeholders::_1);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("vehicle_controller/odom", 1,callback);
  }
  else if (kinematicSourceName=="kinematic")
  {
    auto callback = std::bind(&OdoLocalisationPlugin::processKinematic_,this,std::placeholders::_1);
    kinematic_sub_ = create_subscription<romea_odo_msgs::msg::KinematicMeasureStamped>("vehicle_controller/kinematic", 1,callback);
  }
  else
  {
    RCLCPP_ERROR_STREAM(get_logger(),"Unknown odo_source called : "<<kinematicSourceName);
  }
}

//-----------------------------------------------------------------------------
void OdoLocalisationPlugin::processOdom_(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{

  //std::cout << " processOdom_ "<< std::endl;

  auto twist_msg = std::make_unique<romea_localisation_msgs::msg::ObservationTwist2DStamped>();

  twist_msg->header.frame_id = msg->header.frame_id;
  twist_msg->header.stamp = restamping_ ? get_clock()->now() : rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
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
void OdoLocalisationPlugin::processKinematic_(romea_odo_msgs::msg::KinematicMeasureStamped::ConstSharedPtr msg)
{

  //std::cout << " processKinematic "<< std::endl;

  auto twist_msg = std::make_unique<romea_localisation_msgs::msg::ObservationTwist2DStamped>();

  twist_msg->header.frame_id = msg->header.frame_id;
  twist_msg->header.stamp = restamping_ ? get_clock()->now() : rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
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
