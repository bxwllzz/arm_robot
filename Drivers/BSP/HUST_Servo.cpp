#include "HUST_Servo.hpp"

#include "tim.h"

HUST_Servo servo_yaw(&htim2, TIM_CHANNEL_3, 180);
HUST_Servo servo_pitch(&htim2, TIM_CHANNEL_3, 270);
