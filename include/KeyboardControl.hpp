#ifndef KEYBOARDCONTROL_HPP
#define KEYBOARDCONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <termios.h>

class KeyboardControlNode : public rclcpp::Node {
public:
    KeyboardControlNode();
    ~KeyboardControlNode();

private:
    void timerCallback();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    struct termios old_termios_;

    double linear_speed_;
    double angular_speed_;
};

#endif // KEYBOARDCONTROL_HPP
