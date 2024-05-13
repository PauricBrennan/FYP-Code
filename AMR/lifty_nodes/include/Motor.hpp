#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include "PID.hpp"

class Motor {
        public:
                Motor(void);
                Motor(float,float,float);
                void tick(void);
                int get_control_signal(void);

                int pwm;
                int direction;
                int prev_dir;
                int control_signal;
                float actual_speed;
                float set_point;
                PIDController<float> controller;
};

#endif // MOTOR_HPP_

