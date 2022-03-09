//ros
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

//romea
#include <romea_mobile_base_msgs/msg/kinematic_measure_stamped.hpp>
#include <romea_localisation_msgs/msg/observation_twist2_d_stamped.hpp>

//Eigen
#include <Eigen/Core>

namespace romea {


class OdoLocalisationPlugin : public rclcpp::Node
{
public:

  OdoLocalisationPlugin();

  virtual ~OdoLocalisationPlugin()=default;

  void onInit();

private :

  virtual void processOdom_(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  virtual void processKinematic_(romea_mobile_base_msgs::msg::KinematicMeasureStamped::ConstSharedPtr msg);


private:

   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
   rclcpp::Subscription<romea_mobile_base_msgs::msg::KinematicMeasureStamped>::SharedPtr kinematic_sub_;
   rclcpp::Publisher<romea_localisation_msgs::msg::ObservationTwist2DStamped>::SharedPtr twist_pub_;
   bool restamping_;

};

}
