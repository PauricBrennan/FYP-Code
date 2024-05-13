#ifndef MOTOR_CONTROLLER_NODE_HPP_
#define MOTOR_CONTROLLER_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "fyp_interfaces/msg/mspeeds.hpp"
#include "fyp_interfaces/msg/pw_ms.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "Motor.hpp"


class MotorController : public rclcpp::Node {
  public:
    MotorController();

  private:
    void pwm_callback();
    
    void speed_callback(const fyp_interfaces::msg::Mspeeds);

    void target_callback(const geometry_msgs::msg::Twist::SharedPtr);
    
    Motor m1;
    Motor m2;
    Motor m3;
    Motor m4;
    float wheel_radius;
    float kinematic_model_expression;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<fyp_interfaces::msg::PWMs>::SharedPtr pwms_pub_;
    rclcpp::Subscription<fyp_interfaces::msg::Mspeeds>::SharedPtr actual_speeds>
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};

#endif //MOTOR_CONTROLLER_NODE_HPP_

