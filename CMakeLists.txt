cmake_minimum_required(VERSION 3.5)
project(romea_odo_localisation_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(eigen3_cmake_module REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_core_odo REQUIRED)
find_package(romea_odo_msgs REQUIRED)
find_package(romea_odo_utils REQUIRED)
find_package(romea_localisation_msgs REQUIRED)

add_executable(${PROJECT_NAME}
    src/odo_localisation_plugin.cpp
    src/odo_localisation_plugin_node.cpp)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    romea_core_common
    romea_common_utils
    romea_core_odo
    romea_odo_msgs
    romea_odo_utils
    romea_localisation_msgs
    std_msgs
    nav_msgs
    )

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)


#ament_export_dependencies(eigen3_cmake_module)
#ament_export_dependencies(Eigen3)
#ament_export_dependencies(geometry_msgs)
#ament_export_dependencies(romea_corecommon)
#ament_export_dependencies(romea_common_utils)
#ament_export_dependencies(four_wheel_steering_msgs)
#ament_export_dependencies(ackermann_msgs)
#ament_export_dependencies(romea_core_odo)
#ament_export_dependencies(romea_odo_msgs)
#ament_export_dependencies(romea_odo_utils)
#ament_export_dependencies(romea_localisation_msgs)

#ament_export_include_directories(include)
#ament_export_libraries(${PROJECT_NAME})
#ament_export_targets(${PROJECT_NAME})

#install(TARGETS
#  ${PROJECT_NAME}
#  DESTINATION lib/${PROJECT_NAME})

#install(
#  DIRECTORY include/
#  DESTINATION include
#)

#install(
#  TARGETS ${PROJECT_NAME}
#  EXPORT export_${PROJECT_NAME}
#  LIBRARY DESTINATION lib
#  ARCHIVE DESTINATION lib
#  RUNTIME DESTINATION bin
#  INCLUDES DESTINATION include
#)

#install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

#if(BUILD_TESTING)

#    find_package(ament_lint_auto REQUIRED)
#    ament_lint_auto_find_test_dependencies()
#    add_subdirectory(test)

#endif()

ament_package()

#cmake_minimum_required(VERSION 2.8.3)
#project(romea_odo_localisation_plugin)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

########################
### Find dependencies ##
########################
#set(${PROJECT_NAME}_CATKIN_DEPS
#    roscpp
#    roslib
#    nodelet
#    romea_common_utils
#    romea_odo_msgs
#    romea_localisation_msgs)

#find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
#find_package(romea_common)

####################################
### catkin specific configuration ##
####################################
#catkin_package(
#    LIBRARIES ${PROJECT_NAME}
#    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

############
### build ##
############
#include_directories(
#    SYSTEM ${catkin_INCLUDE_DIRS}
#    SYSTEM ${romea_common_INCLUDE_DIRS})

#add_library(odo_localisation_plugin_nodelet
#  src/odo_localisation_plugin_nodelet.cpp)

#add_executable(odo_localisation_plugin_node
#  src/odo_localisation_plugin_node.cpp)

#target_link_libraries(odo_localisation_plugin_nodelet
#  ${catkin_LIBRARIES}
#  romea_common::romea_common)

#target_link_libraries(odo_localisation_plugin_node
#  ${catkin_LIBRARIES})

