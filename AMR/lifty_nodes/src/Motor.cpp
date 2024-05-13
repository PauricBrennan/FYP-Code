#include "Motor.hpp"

Motor::Motor(void) : controller(0.2, 0.0, 0.0) {
        pwm = 0;
        direction = 0;
        prev_dir = 0;
        control_signal = 0;
        actual_speed = 0.0;
        set_point = 0;

        //controller.setOutputBounds(0.0, 1023.0);
        controller.setMaxIntegralCumulation(30000);
}

Motor::Motor(float p, float i, float d) : controller(p, i, d) {
        pwm = 0;
        direction = 0;
        prev_dir = 0;
        control_signal = 0;
        actual_speed = 0.0;
        set_point = 0;

        //controller.setOutputBounds(0.0, 1023.0);
        controller.setMaxIntegralCumulation(30000);
}

void Motor::tick(void){
        controller.setTarget(set_point);
        controller.setCurrentFeedback(actual_speed);
        controller.tick();
        //perror( (std::to_string(set_point)).c_str() );
        //perror( (std::to_string(actual_speed)).c_str() );
        //perror( (std::to_string(controller.getOutput())).c_str() );
        //perror( (std::to_string(pwm)).c_str() );
}

int Motor::get_control_signal(){
        control_signal += (int) controller.getOutput();

        control_signal = (control_signal > 1023) ? 1023 : control_signal;
        control_signal = (control_signal < 0 || (actual_speed == 0 && set_point>

        //if(prev_dir != direction) {
        //      prev_dir = direction;
                pwm = 0;
                pwm = (direction << 12);
        //} else {
        //      pwm = 0;
        //}
        //perror( (std::to_string(controller.getOutput())).c_str() );

        pwm = pwm | control_signal;

        //perror( (std::to_string(control_signal)).c_str() );

        return pwm;
}


