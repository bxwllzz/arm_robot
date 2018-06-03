#include <servo.hpp>
#include "tim.h"

namespace hustac {

Servo servo_yaw(&htim2, TIM_CHANNEL_3, 180);
Servo servo_pitch(&htim2, TIM_CHANNEL_3, 270);

}
