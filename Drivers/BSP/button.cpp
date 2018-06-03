/*
 * button.cpp
 *
 *  Created on: 2018Äê6ÔÂ3ÈÕ
 *      Author: shuixiang
 */

#include "main.h"

#include "button.hpp"

namespace hustac {

Button button_up(KEY_UP_GPIO_Port, KEY_UP_Pin, GPIO_PIN_RESET, 1000, 200);
Button button_down(KEY_DN_GPIO_Port, KEY_DN_Pin, GPIO_PIN_RESET, 1000, 200);
Button button_start(BUT_START_GPIO_Port, BUT_START_Pin, GPIO_PIN_RESET, 1000, 0);
Button button_stop(BUT_STOP_GPIO_Port, BUT_STOP_Pin, GPIO_PIN_RESET, 1000, 0);
Button button_emergency_stop_left(BUT_STOP1_GPIO_Port, BUT_STOP1_Pin, GPIO_PIN_RESET);
Button button_emergency_stop_right(BUT_STOP2_GPIO_Port, BUT_STOP2_Pin, GPIO_PIN_RESET);

}

