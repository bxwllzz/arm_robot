/*
 * button.hpp
 *
 *  Created on: 2018年6月3日
 *      Author: shuixiang
 */

#pragma once

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

namespace hustac {

class Button {
public:

    GPIO_TypeDef* gpiox;
    const uint16_t gpio_pin;
    const GPIO_PinState active_state;
    const int delay_keystroke;      // ms
    const int delay_hold;           // 触发长按事件的延迟 , =0则不触发长按 ms
    const int delay_hold_repeat;    // 长按后重复触发短按的间隔, =0则不重复短按 ms

    enum class PressState {
        RELEASED,
        PENDING_PRESSED,
        PRESSED,
        PENDING_RELEASED,
    };
    PressState state_press;       // 当前按键状态
    uint32_t last_state_press_change;

    enum class HoldState {
        NO_PRESSED,
        PRESSED,
        PENDING_HOLD,
        HOLDING,
        PENDING_REPEAT,
        REPEATING,
    };
    HoldState state_hold;        // 当前长按状态
    uint32_t last_state_hold_change;

    int count_key_down;     // 按键被按下次数
    int count_key_press;    // 按键被短按次数
    int count_key_hold;     // 按键被长按次数
    int count_key_up;       // 按键被松开次数

    Button(GPIO_TypeDef *_gpiox, uint16_t _gpio_pin,
            GPIO_PinState _active_state = GPIO_PIN_RESET, int _delay_keystroke =
                    20, int _delay_hold = 0, int _delay_hold_repeat = 0) :
            gpiox(_gpiox), gpio_pin(_gpio_pin), active_state(_active_state), delay_keystroke(
                    _delay_keystroke), delay_hold(_delay_hold), delay_hold_repeat(
                    _delay_hold_repeat) {
        reset();
    }

    void reset() {
        state_press = PressState::RELEASED;
        last_state_press_change = HAL_GetTick();
        state_hold = HoldState::NO_PRESSED;
        last_state_hold_change = HAL_GetTick();
        count_key_down = 0;
        count_key_press = 0;
        count_key_hold = 0;
        count_key_up = 0;
    }

    bool is_pin_actived() {
        return HAL_GPIO_ReadPin(gpiox, gpio_pin) == active_state;
    }

    void _FSM_press() {
        bool stop_loop = false;
        while (!stop_loop) {
            switch (state_press) {
            case PressState::RELEASED:
                if (is_pin_actived()) {
                    state_press = PressState::PENDING_PRESSED;
                    last_state_press_change = HAL_GetTick();
                } else {
                    stop_loop = true;
                }
                break;
            case PressState::PENDING_PRESSED:
                if (is_pin_actived()) {
                    if (HAL_GetTick() - last_state_press_change >= (uint32_t)delay_keystroke) {
                        state_press = PressState::PRESSED;
                        last_state_press_change = HAL_GetTick();
                        count_key_down++;
                    } else {
                        stop_loop = true;
                    }
                } else {
                    state_press = PressState::RELEASED;
                    last_state_press_change = HAL_GetTick();
                }
                break;
            case PressState::PRESSED:
                if (!is_pin_actived()) {
                    state_press = PressState::PENDING_RELEASED;
                    last_state_press_change = HAL_GetTick();
                } else {
                    stop_loop = true;
                }
                break;
            case PressState::PENDING_RELEASED:
                if (!is_pin_actived()) {
                    if (HAL_GetTick() - last_state_press_change >= (uint32_t)delay_keystroke) {
                        state_press = PressState::RELEASED;
                        last_state_press_change = HAL_GetTick();
                        count_key_up++;
                    } else {
                        stop_loop = true;
                    }
                } else {
                    state_press = PressState::PRESSED;
                    last_state_press_change = HAL_GetTick();
                }
                break;
            }
        }
    }

    bool is_pressed() {
        return state_press == PressState::PRESSED || state_press == PressState::PENDING_RELEASED;
    }

    void _FSM_simple() {
        bool stop_loop = false;
        while (!stop_loop) {
            switch (state_hold) {
            case HoldState::NO_PRESSED:
                if (is_pressed()) {
                    state_hold = HoldState::PRESSED;
                    last_state_hold_change = HAL_GetTick();
                    count_key_press++;
                } else {
                    stop_loop = true;
                }
                break;
            case HoldState::PRESSED:
                if (!is_pressed()) {
                    state_hold = HoldState::NO_PRESSED;
                    last_state_hold_change = HAL_GetTick();
                } else {
                    stop_loop = true;
                }
                break;
            default:
                stop_loop = true;
            }
        }
    }

    void _FSM_hold() {
        bool stop_loop = false;
        while (!stop_loop) {
            switch (state_hold) {
            case HoldState::NO_PRESSED:
                if (is_pressed()) {
                    state_hold = HoldState::PENDING_HOLD;
                    last_state_hold_change = HAL_GetTick();
                } else {
                    stop_loop = true;
                }
                break;
            case HoldState::PENDING_HOLD:
                if (is_pressed()) {
                    if (HAL_GetTick() - last_state_hold_change >= (uint32_t)delay_hold) {
                        state_hold = HoldState::HOLDING;
                        last_state_hold_change = HAL_GetTick();
                        count_key_hold++;
                    } else {
                        stop_loop = true;
                    }
                } else {
                    state_hold = HoldState::NO_PRESSED;
                    last_state_hold_change = HAL_GetTick();
                    count_key_press++;
                }
                break;
            case HoldState::HOLDING:
                if (!is_pressed()) {
                    state_hold = HoldState::NO_PRESSED;
                    last_state_hold_change = HAL_GetTick();
                } else {
                    stop_loop = true;
                }
                break;
            default:
                stop_loop = true;
            }
        }
    }

    void _FSM_repeat() {
        bool stop_loop = false;
        while (!stop_loop) {
            switch (state_hold) {
            case HoldState::NO_PRESSED:
                if (is_pressed()) {
                    state_hold = HoldState::PENDING_REPEAT;
                    last_state_hold_change = HAL_GetTick();
                    count_key_press++;
                } else {
                    stop_loop = true;
                }
                break;
            case HoldState::PENDING_REPEAT:
                if (is_pressed()) {
                    if (HAL_GetTick() - last_state_hold_change >= (uint32_t)delay_hold) {
                        state_hold = HoldState::REPEATING;
                        last_state_hold_change = HAL_GetTick();
                        if (delay_hold >= delay_hold_repeat) {
                            count_key_press++;
                        }
                    } else {
                        stop_loop = true;
                    }
                } else {
                    state_hold = HoldState::NO_PRESSED;
                    last_state_hold_change = HAL_GetTick();
                }
                break;
            case HoldState::REPEATING:
                if (!is_pressed()) {
                    state_hold = HoldState::NO_PRESSED;
                    last_state_hold_change = HAL_GetTick();
                } else {
                    if (HAL_GetTick() - last_state_hold_change >= (uint32_t)delay_hold_repeat) {
                        last_state_hold_change += delay_hold_repeat;
                        count_key_press++;
                    } else {
                        stop_loop = true;
                    }
                }
                break;
            default:
                stop_loop = true;
            }
        }
    }

    void update() {
        _FSM_press();
        if (delay_hold == 0 && delay_hold_repeat == 0) {
            _FSM_simple();
        } else if (delay_hold > 0 && delay_hold_repeat == 0) {
            _FSM_hold();
        } else if (delay_hold_repeat > 0) {
            _FSM_repeat();
        }
    }

};

extern Button button_up;
extern Button button_down;
extern Button button_start;
extern Button button_stop;
extern Button button_emergency_stop_left;
extern Button button_emergency_stop_right;

}
