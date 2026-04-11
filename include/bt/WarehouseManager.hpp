#ifndef WAREHOUSE_MANAGER_HPP
#define WAREHOUSE_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <random>

class WarehouseManagerNode : public rclcpp::Node {
public:
    WarehouseManagerNode();

private:
    void handleGetPickupTask(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleConfirmLoading(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void handleGetDropoffLocation(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pickup_task_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_loading_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dropoff_location_service_;

    std::mt19937 rng_;
};

#endif // WAREHOUSE_MANAGER_HPP
