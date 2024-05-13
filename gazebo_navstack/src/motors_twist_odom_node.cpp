#include "motors_twist_odom_node.hpp"

using namespace std::chrono_literals;

MotorsTwistOdom::MotorsTwistOdom() : Node("Motors_Odom_Node") {
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("motors_twist", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&MotorsTwistOdom::twist_callback, this));
        
	twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MotorsTwistOdom::dir_callback, this, std::placeholders::_1));

        speeds_sub_ = this->create_subscription<fyp_interfaces::msg::Mspeeds>(
        "Mspeeds", 10, std::bind(&MotorsTwistOdom::speeds_callback, this, std::placeholders::_1));
        
        M1 = 0;
        M2 = 0;
        M3 = 0;
        M4 = 0;
        D1 = 0;
    	D2 = 0;
    	D3 = 0;
    	D4 = 0;
}

void MotorsTwistOdom::dir_callback(const geometry_msgs::msg::Twist::SharedPtr dir){
	float Vx = dir->linear.x;
        float Vy = dir->linear.y;
        float w = dir->angular.z;
        float w1 = 0.0;
        float w2 = 0.0;
        float w3 = 0.0;
        float w4 = 0.0;
        float wheel_radius = 0.0325;
        float kinematic_model_expression = 0.155;


        w1 = ((Vx - Vy - (kinematic_model_expression * w)) / wheel_radius) * 57.3;
        w2 = ((Vx + Vy + (kinematic_model_expression * w)) / wheel_radius) * 57.3;
        w3 = ((Vx + Vy - (kinematic_model_expression * w)) / wheel_radius) * 57.3;
        w4 = ((Vx - Vy + (kinematic_model_expression * w)) / wheel_radius) * 57.3;

        if(w1 != 0) D1 = (w1 > 0) ? 1 : -1;
        if(w2 != 0) D2 = (w2 > 0) ? 1 : -1;
        if(w3 != 0) D3 = (w3 > 0) ? 1 : -1;
        if(w4 != 0) D4 = (w4 > 0) ? 1 : -1;
}

void MotorsTwistOdom::speeds_callback(const fyp_interfaces::msg::Mspeeds speeds) {
	M1 = speeds.m1 * 0.01745;
        M2 = speeds.m2 * 0.01745;
        M3 = speeds.m3 * 0.01745;
        M4 = speeds.m4 * 0.01745;
}

void MotorsTwistOdom::twist_callback() {
	auto msg = std::make_unique<geometry_msgs::msg::Twist>();
	M1 = M1*D1;
	M2 = M2*D2;
	M3 = M3*D3;
	M4 = M4*D4;
	float R = 0.0325;
	float Vx = (M1 + M2 + M3 + M4) * (R / 4);
	float Vy = ( (-1*M1) + M2 + M3 + (-1*M4) ) * (R / 4);
	float W = ( (-1*M1) + M2 + (-1*M3) + M4 ) * 0.0516;
	
	msg->linear.x = Vx;
	msg->linear.y = Vy;
	msg->angular.z = W;
	
	twist_pub_->publish(std::move(msg));
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorsTwistOdom>());
  rclcpp::shutdown();
  return 0;
}
