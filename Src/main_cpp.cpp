
#include <time.h>
#include <cstdio>
#include <inttypes.h>

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

#include "SEGGER_RTT.h"

#include <ros.h>
#include <std_msgs/String.h>

#include "dmabuffer_uart.hpp"
#include "high_resolution_clock.h"
#include "servo.hpp"
#include "button.hpp"
#include "bmx055.hpp"

#include "main_cpp.hpp"

using namespace hustac;

namespace hustac {

DMABuffer_UART<512, 128> terminal(&huart3);

ros::NodeHandle nh;

}

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void ros_receiver_callback(const std_msgs::String& msg) {
    str_msg.data = msg.data;
    chatter.publish(&str_msg);
}
ros::Subscriber<std_msgs::String> receiver("receiver", ros_receiver_callback);

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    nh.getHardware()->dma_buffer.on_tx_dma_complete(huart);
    terminal.on_tx_dma_complete(huart);
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == bmx055_camera.hi2c) {
        bmx055_camera.on_i2c_dma_complete();
    }
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == bmx055_camera.hi2c) {
        bmx055_camera.on_i2c_dma_complete();
    }
}

static uint32_t count_main_loop = 0;
static IMUMeasure imu_measure;
static MagMeasure mag_measure;

void soft_timer_1s_callback(uint32_t count) {
    static uint32_t last_count_imu = 0;
    static uint32_t last_count_mag = 0;

    str_msg.data = "STM32: Hello world!";
    chatter.publish(&str_msg);

    terminal.nprintf<100>("count %d: main %d Hz, imu %d Hz, mag %d Hz\n", count,
            count_main_loop, 
            bmx055_camera.count_imu_measure - last_count_imu,
            bmx055_camera.count_mag_measure - last_count_mag);

    count_main_loop = 0;
    last_count_imu = bmx055_camera.count_imu_measure;
    last_count_mag = bmx055_camera.count_mag_measure;
    
    ros::Time ros_time = nh.now();
    time_t sec = ros_time.sec;
    terminal.write_string(ctime(&sec));
    
//    terminal.nprintf<200>("imu: %" PRIu64 " nsec, accel=(%f, %f, %f), temp=%f, gyro=(%f, %f, %f)\n", 
//        imu_measure.nsec, 
//        imu_measure.accel[0], imu_measure.accel[1], imu_measure.accel[2], 
//        imu_measure.temperature, 
//        imu_measure.gyro[0], imu_measure.gyro[1], imu_measure.gyro[2]
//    );
        
    terminal.nprintf<100>("mag: %" PRIu64 " nsec, (%f, %f, %f)\n", mag_measure.nsec, mag_measure.mag[0], mag_measure.mag[1], mag_measure.mag[2]);

    // terminal.nprintf<50>("SysTick: %" PRIu64 " %" PRIu64 "\n", MY_GetCycleCount(), MY_GetNanoSecFromCycle(MY_GetCycleCount()));
    // terminal.nprintf<50>("DWT: %" PRIu64 " %" PRIu64 "\n", MY_DWTGetCycleCount(), MY_GetNanoSecFromCycle(MY_DWTGetCycleCount()));

    HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
    HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
}

extern "C" void loop_forever(void) {

    MY_DWTTimerInit();

    terminal.init();

    // initialize servo
    terminal.write_string("servo initializing...");
    servo_yaw.start();
    servo_pitch.start();
    terminal.write_string("finished!\n");
    // initialize led
    terminal.write_string("led initializing...");
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    terminal.write_string("finished!\n");
    // initialize imu
//    terminal.write_string("imu initializing...");
//    bmx055_t bmx055;
//    int imu_fail_count = 0;
//    while (bmx055_init(&bmx055) < 0) {
//        imu_fail_count++;
//        terminal.nprintf(20, "failed %d!\n", imu_fail_count);
//        HAL_Delay(50);
//        terminal.write_string("imu initializing...");
//    }
//    terminal.write_string("ok!\n");
    // initialize ros::node_handler
    nh.initNode();
    nh.advertise(chatter);
    nh.subscribe(receiver);

    uint32_t soft_timer_1s = 0;

    while (1) {
        if (HAL_GetTick() > soft_timer_1s) {
            soft_timer_1s += 1000;
            soft_timer_1s_callback(soft_timer_1s / 1000);
        }
        uint8_t buf[128];
        int n = terminal.readsome(buf, 128);
        terminal.write(buf, n);

        nh.spinOnce();

        button_up.update();
        button_down.update();
        button_start.update();
        button_stop.update();
        button_emergency_stop_left.update();
        button_emergency_stop_right.update();

        while (button_up.count_key_press) {
            terminal.write_string("button_up pressed\n");
            button_up.count_key_press--;
        }

        while (button_down.count_key_press) {
            terminal.write_string("button_down pressed\n");
            button_down.count_key_press--;
        }

        while (button_start.count_key_press) {
            terminal.write_string("button_start pressed\n");
            button_start.count_key_press--;
        }

        while (button_start.count_key_hold) {
            terminal.write_string("button_start long pressed\n");
            button_start.count_key_hold--;
        }

        while (button_stop.count_key_press) {
            terminal.write_string("button_stop pressed\n");
            button_stop.count_key_press--;
        }

        while (button_stop.count_key_hold) {
            terminal.write_string("button_stop long pressed\n");
            button_stop.count_key_hold--;
        }
        
        if (button_emergency_stop_left.is_pressed()) {
            terminal.write_string("button_emergency_stop_left hold\n");
        }
        
        if (button_emergency_stop_right.is_pressed()) {
            terminal.write_string("button_emergency_stop_right hold\n");
        }

        bmx055_camera.update();
        if (bmx055_camera.get_imu_measure(imu_measure) > 0) {
        }
        if (bmx055_camera.get_mag_measure(mag_measure) > 0) {
        }
        
        count_main_loop++;
    }

}
