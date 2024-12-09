# romea_ros2_localisation_odo_plugin

This package is an odometry plugin for robot localization, developed within the ROMEA ecosystem for ROS2. It processes data from the mobile base controller to convert it into a simple 2D twist measurement along with its associated covariance.

## ROS2 plugin node description ##

#### 1) Subscribed Topics ####

- **vehicle_controller/odom** (nav_msgs::msg::Odometry)

  This topic is provided by standard vehicle controllers (diff_drive_controller,ackermann_steering_controller, four_wheel_steering_controller),it provides a lot informations like linear angular and speeds and dead reckoning pose but without consistent uncertainties informations.

- **vehicle_controller/kinematic** (romea_msgs::msg::KinematicMeasuredStamped)

  This topic is provided by romea mobile base controllers. It's provides robot kinematic measure and its covariance computed in taking into account odometry data and sensors uncertainties. This topic should be preferred to the other because the uncertainties are reliable.

#### 2) Published Topics ####

- twist (romea_localisation_msgs::ObservationTwist2DStamped)

  Vehicle 2D twist compose by linear velocities along x and y axes + angular velocities around z axis and their covariance 

#### 3) Parameters ####

- **controller_topic** (string, default: kinematic)

    This parameter is used to select odometry source:

    - odom : odometry data are coming from vehicle_controller/odom topic
    - kinematic : odometry data are coming from vehicle_controller/kinematic

- **restamping** (bool, default: false)

    If this parameter is set to true stamp of twist message is equal to computer current time else this stamp is equal odo stamp.  This paremeter will be used when odometry data are coming from a remote master.

## **Usage**

See romea_ros2_localisation_bringup project

## **Contributing**

If you'd like to contribute to this project, here are some guidelines:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes.
4. Write tests to cover your changes.
5. Run the tests to ensure they pass.
6. Commit your changes.
7. Push your changes to yo

## License

This project is released under the Apache License 2.0. See the LICENSE file for details.

### Authors

The romea_ros2_localisation_odo_plugin project was developed by **Jean Laneurit** in the context of BaudetRob2 ANR project.

### Contact

If you have any questions or comments about romea_ros2_localisation_odo_plugin project, please contact **[Jean Laneurit](mailto:jean.laneurit@inrae.fr)** 
