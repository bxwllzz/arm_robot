
#include <cstdio>

#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <ros.h>
#include <std_msgs/String.h>

#include "HUST_Servo.hpp"
#include "SEGGER_RTT.h"
#include "bmx055.h"
#include "BufferUARTDMA.hpp"

BufferUARTDMA uart_logger(&huart3, 512, 128);

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if (huart == nh.getHardware()->huart) {
        nh.getHardware()->_flush();
    }
}

void soft_timer_1s_callback(uint32_t count) {
    str_msg.data = "STM32: Hello world!";
    chatter.publish(&str_msg);
    static char strbuf[10];
    sprintf(strbuf, "count %d\n", (int)count);
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)strbuf, strlen(strbuf));
    HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
    HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
}

extern "C" void loop_forever() {

    uart_logger.init();

    // initialize servo
    uart_logger.write_string("servo initializing...");
    servo_yaw.start();
    servo_pitch.start();
    uart_logger.write_string("finished!\n");
    // initialize led
    uart_logger.write_string("led initializing...");
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    uart_logger.write_string("finished!\n");
    // initialize imu
    uart_logger.write_string("imu initializing...");
    bmx055_t bmx055;
    int imu_fail_count = 0;
    while (bmx055_init(&bmx055) < 0) {
        imu_fail_count++;
        uart_logger.nprintf(20, "failed %d!\n", imu_fail_count);
        HAL_Delay(50);
        uart_logger.write_string("imu initializing...");
    }
    uart_logger.write_string("ok!\n");
    // initialize ros::node_handler
    nh.initNode();
    nh.advertise(chatter);

    uint32_t soft_timer_1s = 0;

    while (1) {
        if (HAL_GetTick() > soft_timer_1s) {
            soft_timer_1s += 1000;
            soft_timer_1s_callback(soft_timer_1s / 1000);
        }
        nh.spinOnce();
    }

}
