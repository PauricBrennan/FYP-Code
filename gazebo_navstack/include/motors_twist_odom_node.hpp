#ifndef MOTORS_TWIST_ODOM_NODE_HPP_
#define MOTORS_TWIST_ODOM_NODE_HPP_


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "fyp_interfaces/msg/mspeeds.hpp"

class MotorsTwistOdom : public rclcpp::Node {
	public:
		MotorsTwistOdom();

		void speeds_callback(const fyp_interfaces::msg::Mspeeds);
		void dir_callback(const geometry_msgs::msg::Twist::SharedPtr);
		void twist_callback();

		
	private:
		rclcpp::TimerBase::SharedPtr timer_;
    		rclcpp::Subscription<fyp_interfaces::msg::Mspeeds>::SharedPtr speeds_sub_;
    		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    		int D1;
    		int D2;
    		int D3;
    		int D4;
		int M1;
		int M2;
		int M3;
		int M4;
};

//double map(double value, double from_min, double from_max, double to_min, double);
			
#endif //MOTORS_TWIST_ODOM_NODE_HPP_


