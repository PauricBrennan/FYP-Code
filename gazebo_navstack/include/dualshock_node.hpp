#ifndef DUALSHOCK_NODE_HPP_
#define DUALSHOCK_NODE_HPP_


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

class Dualshock : public rclcpp::Node {
	public:
		Dualshock();

		void joy_callback(const sensor_msgs::msg::Joy::SharedPtr);

		void twist_callback();
		
		float Vx;
		float Vy;
		float W;
		
	private:
		rclcpp::TimerBase::SharedPtr timer_;
    		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

};

//double map(double value, double from_min, double from_max, double to_min, double);
			
#endif //DUALSHOCK_NODE_HPP_
