#include "bt/WarehouseManager.hpp"

#include <chrono>
#include <thread>

WarehouseManagerNode::WarehouseManagerNode()
    : Node("warehouse_manager"),
      rng_(std::random_device{}())
{
    pickup_task_service_ = this->create_service<std_srvs::srv::Trigger>(
        "get_pickup_task",
        std::bind(&WarehouseManagerNode::handleGetPickupTask, this,
                  std::placeholders::_1, std::placeholders::_2));

    confirm_loading_service_ = this->create_service<std_srvs::srv::Trigger>(
        "confirm_loading",
        std::bind(&WarehouseManagerNode::handleConfirmLoading, this,
                  std::placeholders::_1, std::placeholders::_2));

    dropoff_location_service_ = this->create_service<std_srvs::srv::Trigger>(
        "get_dropoff_location",
        std::bind(&WarehouseManagerNode::handleGetDropoffLocation, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "WarehouseManager ready. Services: "
        "/get_pickup_task, /confirm_loading, /get_dropoff_location");
}

void WarehouseManagerNode::handleGetPickupTask(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::uniform_int_distribution<int> dist(1, 3);
    int manipulator_id = dist(rng_);

    response->success = true;
    response->message = std::to_string(manipulator_id);

    RCLCPP_INFO(this->get_logger(), "Pickup task assigned: manipulator %d", manipulator_id);
}

void WarehouseManagerNode::handleConfirmLoading(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::uniform_int_distribution<int> dist(2000, 5000);
    int wait_ms = dist(rng_);

    RCLCPP_INFO(this->get_logger(), "Loading in progress... (%.1f s)", wait_ms / 1000.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));

    response->success = true;
    response->message = "Loading complete";

    RCLCPP_INFO(this->get_logger(), "Loading complete after %.1f s", wait_ms / 1000.0);
}

void WarehouseManagerNode::handleGetDropoffLocation(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::uniform_int_distribution<int> letter_dist(0, 3);  // A-D
    std::uniform_int_distribution<int> number_dist(1, 2);  // 1-2
    char storage_letter = 'A' + letter_dist(rng_);
    int storage_number = number_dist(rng_);

    response->success = true;
    response->message = std::string(1, storage_letter) + std::to_string(storage_number);

    RCLCPP_INFO(this->get_logger(), "Dropoff assigned: storage %s", response->message.c_str());
}
