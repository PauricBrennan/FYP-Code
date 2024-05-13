#include "dualshock_node.hpp"

using namespace std::chrono_literals;

Dualshock::Dualshock() : Node("Dualshock_Twist") {
        twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_nav", 10);
        timer_ = this->create_wall_timer(250ms, std::bind(&Dualshock::twist_callback, this));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&Dualshock::joy_callback, this, std::placeholders::_1));
        
        Vx = 0.0;
        Vy = 0.0;
        W = 0.0;
        
        //command = 0;
        //fork_state = true;
}

void Dualshock::joy_callback(const sensor_msgs::msg::Joy::SharedPtr controls) {
	Vx = controls->axes[3];
	Vy = controls->axes[2];
	W = controls->axes[0] * 2;
	
	
	//BUTTONS
	if(controls->buttons[0]){
	
	} else if(controls->buttons[1]) {
	
	} else if(controls->buttons[2]) {
	
	} else if(controls->buttons[3]) {
	
	}
	
	if(controls->buttons[11]){
	
	} else if(controls->buttons[12]) {
	
	} 
}

void Dualshock::twist_callback() {
	auto msg = std::make_unique<geometry_msgs::msg::Twist>();
	
	
	msg->linear.x = Vx;
	msg->linear.y = Vy;
	msg->angular.z = W;
	twist_pub_->publish(std::move(msg));
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dualshock>());
  rclcpp::shutdown();
  return 0;
}
