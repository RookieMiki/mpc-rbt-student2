#include <chrono>
#include <functional>

#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

#include "KeyboardControl.hpp"

using namespace std::chrono_literals;

KeyboardControlNode::KeyboardControlNode() : rclcpp::Node("keyboard_control_node") {
    this->declare_parameter("linear_speed", 0.5);
    this->declare_parameter("angular_speed", 0.5);

    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(10ms, std::bind(&KeyboardControlNode::timerCallback, this));

    tcgetattr(STDIN_FILENO, &old_termios_);
    struct termios new_termios = old_termios_;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    RCLCPP_INFO(get_logger(), "Keyboard Control node started.");
    RCLCPP_INFO(get_logger(), "Use arrow keys to control robot.");
    RCLCPP_INFO(get_logger(), "Speed range: 0.0 to 1.5");
}

KeyboardControlNode::~KeyboardControlNode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void KeyboardControlNode::timerCallback() {
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();

    if (linear_speed_ < 0.0) linear_speed_ = 0.0;
    if (linear_speed_ > 1.5) linear_speed_ = 1.5;

    if (angular_speed_ < 0.0) angular_speed_ = 0.0;
    if (angular_speed_ > 1.5) angular_speed_ = 1.5;

    geometry_msgs::msg::Twist twist{};
    char c;

    if (read(STDIN_FILENO, &c, 1) == 1) {
        if (c == '\033') {
            char seq1, seq2;

            if (read(STDIN_FILENO, &seq1, 1) == 1 && read(STDIN_FILENO, &seq2, 1) == 1) {
                if (seq1 == '[') {
                    switch (seq2) {
                        case 'A':
                            twist.linear.x = linear_speed_;
                            break;
                        case 'B':
                            twist.linear.x = -linear_speed_;
                            break;
                        case 'C':
                            twist.angular.z = -angular_speed_;
                            break;
                        case 'D':
                            twist.angular.z = angular_speed_;
                            break;
                        default:
                            return;
                    }

                    twist_publisher_->publish(twist);
                }
            }
        }
    }
}
