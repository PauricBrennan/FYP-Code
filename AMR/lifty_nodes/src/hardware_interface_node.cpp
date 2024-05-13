#include "hardware_interface_node.hpp"

using namespace std::chrono_literals;

HardwareInterface::HardwareInterface(int i2c) : Node("HardWare_Interface"){
        //Publisheer
        publisher_ = this->create_publisher<fyp_interfaces::msg::Mspeeds>("Mspe>
        timer_ = this->create_wall_timer(
        25ms, std::bind(&HardwareInterface::timer_callback, this));

        subscription_ = this->create_subscription<fyp_interfaces::msg::PWMs>(
        "PWMs", 10, std::bind(&HardwareInterface::topic_callback, this, std::pl>

        file = i2c;
}

void HardwareInterface::timer_callback(){
      auto message = fyp_interfaces::msg::Mspeeds();
      message.m1 = get_motor_speed(m1_address, file);
      message.m2 = get_motor_speed(m2_address, file);
      message.m3 = get_motor_speed(m3_address, file);
      message.m4 = get_motor_speed(m4_address, file);

     // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.m4);
      publisher_->publish(message);
}

void HardwareInterface::topic_callback(const fyp_interfaces::msg::PWMs pwm){
      set_pwm(pwm.m1, m1_address, file);
      set_pwm(pwm.m2, m2_address, file);
      set_pwm(pwm.m3, m3_address, file);
      set_pwm(pwm.m4, m4_address, file);
}

float HardwareInterface::get_motor_speed(int address, int file){
        unsigned char data[3] = {0,0,0};
        unsigned char data_reset[1] = {0};
        unsigned int period = 0;
        unsigned int value = 0;
        float error = 0;
        float frequency = 0;
        float degrees_per_pulse = 0.962567; 
	
	
	if (ioctl(file, I2C_SLAVE, address) < 0) {
           perror("Failed to set I2C address");
           return -1;
        }       

        if (read(file, data, sizeof(data)) != sizeof(data)) {
            perror("Failed to read from I2C bus");
            return -1;
        }

        value = data[0];
        period = period | value;
        value = data[1];
        period = period | (value << 8);
        value = data[2];
        period = period | (value << 16);

        if(period > 200000){
                std::string command = "i2cget -y 1 " + std::to_string(address);
                int result = system(command.c_str());

                if (read(file, data, sizeof(data)) != sizeof(data)) {
                    perror("Failed to read from I2C bus");
                    return -1;
                }

                value = data[0];
                period = period | value;
                value = data[1];
                period = period | (value << 8);
                value = data[2];
                period = period | (value << 16);
        }
        
        if(period == 0) return 0;

        frequency = (float)((double) 1.0 / ((double) (period * ((double) 0.0000>
        error = (0.01*frequency) + 0.15;
        float output = (frequency+error)*degrees_per_pulse;
        if(address == 11){
                //perror(std::to_string(data[2]).c_str());
                //perror(std::to_string(data[1]).c_str());
                //perror(std::to_string(data[0]).c_str());
        //      perror("Whats The ISSUE????");
        }

        return (frequency + error) * degrees_per_pulse;
}

int HardwareInterface::set_pwm(int pwm, int address, int file){

        if (ioctl(file, I2C_SLAVE, address) < 0) {
           perror("Failed to set I2C address");
           return -1;
        }        

        unsigned char buffer[2] = {(pwm & 255), (pwm >> 8)};

        if (write(file, buffer, sizeof(buffer)) != sizeof(buffer)) {
           perror("Failed to write to I2C bus");
        }

        return 1;
}

