# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: pooyanjamshidi, marioney, tmxkn1
# SPDX-FileCopyrightText: Czech Technical University in Prague
#
# Original file from https://github.com/tmxkn1/brass_gazebo_battery edited by Martin Pecka:
# - renamed to gazebo_ros_battery
# - cleaned up the code

cmake_minimum_required(VERSION 3.10.2)
project(gazebo_ros_battery)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_plugins
  gazebo_ros
  message_generation
  roscpp
)

add_service_files(FILES
  SetCharge.srv
  SetCharging.srv
  SetChargingRate.srv
  SetCoef.srv
  SetLoad.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_discharge SHARED src/battery_discharge.cpp)
add_dependencies(${PROJECT_NAME}_discharge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_discharge ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_consumer SHARED src/battery_consumer.cpp)
add_dependencies(${PROJECT_NAME}_consumer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_consumer ${catkin_LIBRARIES})

add_library(gazebo_ros_cmd_vel_battery_consumer SHARED src/cmd_vel_consumer.cpp)
add_dependencies(gazebo_ros_cmd_vel_battery_consumer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_cmd_vel_battery_consumer ${catkin_LIBRARIES})

add_library(gazebo_ros_motor_battery_consumer SHARED src/motor_consumer.cpp)
add_dependencies(gazebo_ros_motor_battery_consumer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ros_motor_battery_consumer ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_consumer ${PROJECT_NAME}_discharge gazebo_ros_cmd_vel_battery_consumer gazebo_ros_motor_battery_consumer
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    find_package(roslint REQUIRED)

    # catkin_lint - checks validity of package.xml and CMakeLists.txt
    # ROS buildfarm calls this without any environment and with empty rosdep cache,
    # so we have problems reading the list of packages from env
    # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
    if(DEFINED ENV{ROS_HOME})
        #catkin_lint: ignore_once env_var
        set(ROS_HOME "$ENV{ROS_HOME}")
    else()
        #catkin_lint: ignore_once env_var
        set(ROS_HOME "$ENV{HOME}/.ros")
    endif()

    #catkin_lint: ignore_once env_var
    if(DEFINED ENV{ROS_ROOT} AND EXISTS "${ROS_HOME}/rosdep/sources.cache")
        roslint_custom(catkin_lint -W2 .)
    endif()

    roslint_add_test()
endif()