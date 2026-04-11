cmake_minimum_required(VERSION 3.5)
project(mpc_rbt_student)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# Enforce stricter compilation rules
add_compile_options(-Wall -Wextra -Wpedantic)

# Make sure all required dependencies are locatable
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED) 
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mpc_rbt_simulator REQUIRED)

# Define include directories for the compiler
include_directories(PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# Build and install libraries
set(dependencies
        rclcpp
        rclcpp_action
        geometry_msgs # change here
        sensor_msgs
        nav_msgs # already was
        nav2_msgs # already was
        tf2
        tf2_ros
        tf2_geometry_msgs
        mpc_rbt_simulator
)

add_library(${PROJECT_NAME} SHARED
 			src/KeyboardControl.cpp
 			src/Localization.cpp
 			src/Planning.cpp
 			src/MotionControl.cpp)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Build and install executables
# * * * * * * * * * * * * * * * * *
# *      Lab 04_keyboard_control
# * * * * * * * * * * * * * * * * *
add_executable(keyboard_control src/keyboard_control_node.cpp)
target_link_libraries(keyboard_control ${PROJECT_NAME})
install(TARGETS keyboard_control DESTINATION lib/${PROJECT_NAME})

# * * * * * * * * * * * * * * * * *
# *      Lab 05_localization
# * * * * * * * * * * * * * * * * *
add_executable(localization src/localization_node.cpp)
target_link_libraries(localization ${PROJECT_NAME})
install(TARGETS localization DESTINATION lib/${PROJECT_NAME})

# * * * * * * * * * * * * * * * * *
# *      Lab 06_planning
# * * * * * * * * * * * * * * * * *
add_executable(planning src/planning_node.cpp)
target_link_libraries(planning ${PROJECT_NAME})
install(TARGETS planning DESTINATION lib/${PROJECT_NAME})

# * * * * * * * * * * * * * * * * *
# * Lab 07_control
# * * * * * * * * * * * * * * * * *
add_executable(motion_control src/motion_control_node.cpp)
target_link_libraries(motion_control ${PROJECT_NAME})
install(TARGETS motion_control DESTINATION lib/${PROJECT_NAME})

# * * * * * * * * * * * * * * * * *
# *    Lab 08_behavior trees
# * * * * * * * * * * * * * * * * *


# Install additional directories and files
install(DIRECTORY
        launch
        rviz
        DESTINATION share/${PROJECT_NAME}/
)

# Set up testing
#TODO(linting): enable linting tests only if we want to enforce strict ROS code formatting, otherwise they will fail
#if (BUILD_TESTING)
#    #    find_package(ament_lint_auto REQUIRED)
#    find_package(ament_cmake_gtest REQUIRED)
#    #    ament_lint_auto_find_test_dependencies()
#    enable_testing()
#
#    add_subdirectory(tests)
#endif ()

# ROS exports
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_package()
