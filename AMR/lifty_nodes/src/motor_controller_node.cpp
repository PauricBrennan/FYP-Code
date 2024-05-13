#include "motor_controller_node.hpp"

using namespace std::chrono_literals;

MotorController::MotorController() : Node("Motor_Controller") {
        //Publisheer
        pwms_pub_ = this->create_publisher<fyp_interfaces::msg::PWMs>("PWMs", 1>
        timer_ = this->create_wall_timer(160ms, std::bind(&MotorController::pwm>

        actual_speeds_sub_ = this->create_subscription<fyp_interfaces::msg::Msp>
        "Mspeeds", 30, std::bind(&MotorController::speed_callback, this, std::p>

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_nav", 10, std::bind(&MotorController::target_callback, this, s>

        m1 = Motor();
        m2 = Motor();
        m3 = Motor();
        m4 = Motor();

        wheel_radius = 0.0325;
        kinematic_model_expression = 0.155;
}

void MotorController::pwm_callback() {
      auto message = fyp_interfaces::msg::PWMs();

      m1.tick();
      m2.tick();
      m3.tick();
      m4.tick();

      message.m1 = m1.get_control_signal();
      message.m2 = m2.get_control_signal();
      message.m3 = m3.get_control_signal();
      message.m4 = m4.get_control_signal();

      pwms_pub_->publish(message);
      //if(m1.controller.isEnabled()) perror("Is Enabled");
      //if(m1.controller.getP() == 2.0) perror("it is 2");
      //if(m1.controller.getTarget() == 855) perror("Set point works");
      //perror( "Publishing" );
}

void MotorController::speed_callback(const fyp_interfaces::msg::Mspeeds actual_>
        m1.actual_speed = actual_speeds.m1;
        m2.actual_speed = actual_speeds.m2;
        m3.actual_speed = actual_speeds.m3;
        m4.actual_speed = actual_speeds.m4;
        //perror( (std::to_string(actual_speeds.m1)).c_str() );
}

void MotorController::target_callback(const geometry_msgs::msg::Twist::SharedPt>
        float Vx = target->linear.x;
        float Vy = target->linear.y;
        float w = target->angular.z;
        float w1 = 0.0;
        float w2 = 0.0;
        float w3 = 0.0;
        float w4 = 0.0;



        w1 = ((Vx - Vy - (kinematic_model_expression * w)) / wheel_radius) * 57>
        w2 = ((Vx + Vy + (kinematic_model_expression * w)) / wheel_radius) * 57>
        w3 = ((Vx + Vy - (kinematic_model_expression * w)) / wheel_radius) * 57>
        w4 = ((Vx - Vy + (kinematic_model_expression * w)) / wheel_radius) * 57>



        m1.set_point = std::abs(w1);
        m2.set_point = std::abs(w2);
        m3.set_point = std::abs(w3);
        m4.set_point = std::abs(w4);

        //perror(std::to_string(w1).c_str());
        //perror(std::to_string(w2).c_str());
        //perror(std::to_string(w3).c_str());
        //perror(std::to_string(m1.set_point).c_str());
        //perror((std::to_string(m1.set_point)).c_str());
        //perror((std::to_string(m2.set_point)).c_str());
        //perror((std::to_string(m3.set_point)).c_str());
        //perror((std::to_string(m4.set_point)).c_str());
        if(w1 != 0) m1.direction = (w1 < 0) ? 1 : 2;
        if(w2 != 0) m2.direction = (w2 < 0) ? 1 : 2;
        if(w3 != 0) m3.direction = (w3 < 0) ? 1 : 2;
        if(w4 != 0) m4.direction = (w4 < 0) ? 1 : 2;
        //perror(std::to_string(m1.direction).c_str());
        //perror(std::to_string(m2.direction).c_str());
        //perror(std::to_string(m3.direction).c_str());
        //perror(std::to_string(m4.direction).c_str());

}



int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorController>());
  rclcpp::shutdown();
  return 0;
}

