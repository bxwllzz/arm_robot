#pragma once

#include <cmath>
#include <algorithm>

#include "stm32f1xx_hal.h"

#include "accel_limited_controller.hpp"

namespace hustac {

class Servo {
public:
    Servo(TIM_HandleTypeDef* _htim, uint32_t _channel, float _range,
            float init_angle = 0, int _update_interval_ms = 10) :
            htim(_htim), channel(_channel), range(_range), controller(
                    init_angle, 0, _update_interval_ms) {
        controller.max_vel = 15;
        controller.max_accel = 30;
    }
    // 启动舵机
    void start() {
        HAL_TIM_PWM_Start(htim, channel);
        _set_pulse_width(_angle2width(controller.current_pos));
    }
    // 停止舵机
    void stop() {
        HAL_TIM_PWM_Stop(htim, channel);
    }
    // 设定期望角度
    float set_angle(float angle) {
        angle = std::min(angle, range / 2);
        angle = std::max(angle, -range / 2);
        controller.target_pos = angle;
        return angle;
    }
    // 获取当前角度
    float get_angle() {
        return controller.current_pos;
    }
    // 设置最大速度
    float set_max_velocity(float max_velocity) {
        controller.max_vel = max_velocity;
        return max_velocity;
    }
    // 控制更新
    bool update() {
        if (controller.update()) {
            controller.current_vel = controller.output_vel;
            controller.current_pos = controller.output_pos;
            _set_pulse_width(_angle2width(controller.current_pos));
            return true;
        } else {
            return false;
        }
    }
private:
    int _angle2width(float angle) {
        int ms = (int) std::round((angle / range + 0.5) * (max_width_ms - min_width_ms) + min_width_ms);
        ms = std::min(ms, max_width_ms);
        ms = std::max(ms, min_width_ms);
        return ms;
    }
    void _set_pulse_width(int ms) {
        __HAL_TIM_SET_COMPARE(htim, channel, ms);
    }
public:
    TIM_HandleTypeDef* htim; // TIM定时器
    uint32_t channel;		// TIM通道
    float range;			// 旋转范围/deg
    AccelLimitedController controller;
    int min_width_ms = 500;		// 最小脉宽/ms
    int max_width_ms = 2500;		// 最大脉宽/ms

};

extern Servo servo_yaw;
extern Servo servo_pitch;

}
