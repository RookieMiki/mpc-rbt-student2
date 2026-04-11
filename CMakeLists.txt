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

# TODO (Lab 08): Odkomentujte následující závislosti pro behavior trees
# find_package(std_srvs REQUIRED)
# find_package(behaviortree_cpp REQUIRED)
# find_package(behaviortree_ros2 REQUIRED)

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

# TODO (Lab 08): Přidejte std_srvs do dependencies výše a odkomentujte bt_dependencies
# set(bt_dependencies
#         ${dependencies}
#         std_srvs
#         behaviortree_cpp
#         behaviortree_ros2
# )


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
# TODO (Lab 08): Odkomentujte celou sekci níže po nainstalování BT závislostí.

# # Warehouse manager (provided)
# add_library(warehouse_manager_lib SHARED src/bt/WarehouseManager.cpp)
# ament_target_dependencies(warehouse_manager_lib ${dependencies} std_srvs)
# add_executable(warehouse_manager src/bt/warehouse_manager_node.cpp)
# target_link_libraries(warehouse_manager warehouse_manager_lib ${PROJECT_NAME})
# install(TARGETS warehouse_manager_lib ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
# install(TARGETS warehouse_manager DESTINATION lib/${PROJECT_NAME})
#
# # BT server executable
# add_executable(bt_server src/bt/bt_server.cpp)
# ament_target_dependencies(bt_server ${bt_dependencies})
# set_target_properties(bt_server PROPERTIES CXX_STANDARD 17)
# install(TARGETS bt_server DESTINATION lib/${PROJECT_NAME})
#
# # BT plugins (shared libraries)
# set(BT_PLUGIN_TARGETS
#   navigate_to_pose_action_bt_node
#   get_task_service_bt_node
#   confirm_loading_service_bt_node
#   get_dropoff_service_bt_node
#   lookup_pose_bt_node
# )
#
# add_library(navigate_to_pose_action_bt_node SHARED src/bt/NavigateToPoseAction.cpp)
# add_library(get_task_service_bt_node SHARED src/bt/GetTaskService.cpp)
# add_library(confirm_loading_service_bt_node SHARED src/bt/ConfirmLoadingService.cpp)
# add_library(get_dropoff_service_bt_node SHARED src/bt/GetDropoffService.cpp)
# add_library(lookup_pose_bt_node SHARED src/bt/LookupPose.cpp)
#
# foreach(target ${BT_PLUGIN_TARGETS})
#   ament_target_dependencies(${target} ${bt_dependencies})
#   target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
#   set_target_properties(${target} PROPERTIES CXX_STANDARD 17)
# endforeach()
#
# install(TARGETS
#   navigate_to_pose_action_bt_node
#   get_task_service_bt_node
#   confirm_loading_service_bt_node
#   get_dropoff_service_bt_node
#   lookup_pose_bt_node
#   LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
#   ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
#   RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
# )

# Install additional directories and files
install(DIRECTORY
        launch
        rviz
        # TODO (Lab 08): Odkomentujte po implementaci BT
        # behavior_trees
        # config
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
