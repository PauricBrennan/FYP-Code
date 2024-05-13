#ifndef HARDWARE_INTERFACE_NODE_HPP_
#define HARDWARE_INTERFACE_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "rclcpp/rclcpp.hpp"
#include "fyp_interfaces/msg/mspeeds.hpp"
#include "fyp_interfaces/msg/pw_ms.hpp"


class HardwareInterface : public rclcpp::Node {
  public:
    HardwareInterface(int);

  private:
    void timer_callback();
    
    void topic_callback(const fyp_interfaces::msg::PWMs);
    
    float get_motor_speed(int, int);
    
    int set_pwm(int, int, int);
    
    
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<fyp_interfaces::msg::Mspeeds>::SharedPtr publisher_;
    rclcpp::Subscription<fyp_interfaces::msg::PWMs>::SharedPtr subscription_;

    int file;
    
    int m1_address = 0x8;
    int m2_address = 0x9;
    int m3_address = 0xa;
    int m4_address = 0xb;
    
    int m1_pwm;
    int m2_pwm;
    int m3_pwm;
    int m4_pwm;
};

#endif //HARDWARE_INTERFACE_NODE_HPP_

