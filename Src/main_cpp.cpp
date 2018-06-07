#include <time.h>
#include <inttypes.h>
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

#include "SEGGER_RTT.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/JointState.h>
#include <arm_robot_msgs/Imu.h>
#include <arm_robot_msgs/MagneticField.h>

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

sensor_msgs::Temperature temp_camera_msg;
ros::Publisher pub_temp_camera("temp_camera", &temp_camera_msg);

sensor_msgs::JointState servo_state_msg;
ros::Publisher pub_servo_state("/servo_joint_states", &servo_state_msg);

arm_robot_msgs::Imu imu_camera_msg;
ros::Publisher pub_imu_camera("imu_camera", &imu_camera_msg);

arm_robot_msgs::MagneticField mag_camera_msg;
ros::Publisher pub_mag_camera("mag_camera", &mag_camera_msg);

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

static IMUMeasure imu_measure;
static MagMeasure mag_measure;
static uint32_t count_send_mag = 0;
static uint32_t count_send_imu = 0;

static void handle_debug() {
    static uint32_t count_main_loop = 0;
    static uint32_t soft_timer_1s = 0;
    
    if (HAL_GetTick() > soft_timer_1s) {
        soft_timer_1s += 1000;
        uint32_t count = soft_timer_1s / 1000;

        str_msg.data = "STM32: Hello world!";
        chatter.publish(&str_msg);
        
        static uint32_t last_count_imu = 0;
        static uint32_t last_count_mag = 0;

        terminal.nprintf<100>("count %d: main %d Hz, imu %d Hz (%d), mag %d Hz(%d)\n",
                count, count_main_loop,
                bmx055_camera.count_imu_measure - last_count_imu, count_send_imu, 
                bmx055_camera.count_mag_measure - last_count_mag, count_send_mag);

        count_main_loop = 0;
        last_count_imu = bmx055_camera.count_imu_measure;
        last_count_mag = bmx055_camera.count_mag_measure;
        count_send_mag = 0;
        count_send_imu = 0;
        
//        ros::Time ros_time = nh.now();
//        time_t sec = ros_time.sec;
//        terminal.write_string(ctime(&sec));
        
        //    terminal.nprintf<200>("imu: %" PRIu64 " nsec, accel=(%f, %f, %f), temp=%f, gyro=(%f, %f, %f)\n",
        //        imu_measure.nsec,
        //        imu_measure.accel[0], imu_measure.accel[1], imu_measure.accel[2],
        //        imu_measure.temperature,
        //        imu_measure.gyro[0], imu_measure.gyro[1], imu_measure.gyro[2]
        //    );

//        terminal.nprintf<100>("mag: %" PRIu64 " nsec, (%f, %f, %f)\n",
//                mag_measure.nsec, mag_measure.mag[0], mag_measure.mag[1],
//                mag_measure.mag[2]);

        // terminal.nprintf<50>("SysTick: %" PRIu64 " %" PRIu64 "\n", MY_GetCycleCount(), MY_GetNanoSecFromCycle(MY_GetCycleCount()));
        // terminal.nprintf<50>("DWT: %" PRIu64 " %" PRIu64 "\n", MY_DWTGetCycleCount(), MY_GetNanoSecFromCycle(MY_DWTGetCycleCount()));

        HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
        HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
    }
    
    count_main_loop++;
}

static void update_button() {
    button_up.update();
    button_down.update();
    button_start.update();
    button_stop.update();
    button_emergency_stop_left.update();
    button_emergency_stop_right.update();
}

static void handle_button() {
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
}

static void update_imu() {
    bmx055_camera.update();
}

static void handle_imu() {
    int ret;
    static uint32_t timer_pub_temp_camera = 0;
    if (bmx055_camera.get_imu_measure(imu_measure) > 0) {
        imu_camera_msg.header.seq = imu_measure.seq;
        imu_camera_msg.header.stamp = nh.fromNSec(imu_measure.nsec);
//        imu_camera_msg.header.frame_id = "";
        imu_camera_msg.angular_velocity.x = imu_measure.gyro[0];
        imu_camera_msg.angular_velocity.y = imu_measure.gyro[1];
        imu_camera_msg.angular_velocity.z = imu_measure.gyro[2];
        imu_camera_msg.linear_acceleration.x = imu_measure.accel[0];
        imu_camera_msg.linear_acceleration.y = imu_measure.accel[1];
        imu_camera_msg.linear_acceleration.z = imu_measure.accel[2];
        ret = pub_imu_camera.publish(&imu_camera_msg);
        if (ret < 0) {
            terminal.write_string(
                    "node_handler: failed to publish imu_camera (");
            terminal.nprintf("%d)\n", ret);
        } else if (nh.connected()) {
            count_send_imu++;
        }
        temp_camera_msg.header.seq = imu_measure.seq;
        temp_camera_msg.header.stamp = nh.fromNSec(imu_measure.nsec);
//        temp_camera_msg.header.frame_id = "";
        temp_camera_msg.temperature = imu_measure.temperature;
//        temp_camera_msg.variance = 0;
        if (HAL_GetTick() > timer_pub_temp_camera) {
            // publish temperature every 1s
            ret = pub_temp_camera.publish(&temp_camera_msg);
            if (ret >= 0) {
                timer_pub_temp_camera += 1000;
            } else {
                terminal.write_string(
                        "node_handler: failed to publish temp_camera (");
                terminal.nprintf("%d)\n", ret);
            }
        }
    }
    
    if (bmx055_camera.get_mag_measure(mag_measure) > 0) {
        mag_camera_msg.header.seq = mag_measure.seq;
        mag_camera_msg.header.stamp = nh.fromNSec(mag_measure.nsec);
//        mag_camera_msg.header.frame_id = "";
        mag_camera_msg.magnetic_field.x = mag_measure.mag[0];
        mag_camera_msg.magnetic_field.y = mag_measure.mag[1];
        mag_camera_msg.magnetic_field.z = mag_measure.mag[2];
        ret = pub_mag_camera.publish(&mag_camera_msg);
        if (ret < 0) {
            terminal.write_string(
                    "node_handler: failed to publish mag_camera (");
            terminal.nprintf("%d)\n", ret);
        } else if (nh.connected()) {
            count_send_mag++;
        }
    }
}

static void update_servo() {
    static uint32_t timer_100ms = 0;
    static uint32_t count = 0;
    static const char* name[2] = {"camera_yaw_joint", "camera_pitch_joint"};
    static float position[2];
    if (HAL_GetTick() > timer_100ms) {
        timer_100ms += 100;
        servo_state_msg.header.seq = count;
        servo_state_msg.header.stamp = nh.now();
        servo_state_msg.name_length = 2;
        servo_state_msg.name = (char**)name;
        servo_state_msg.position_length = 2;
        position[0] = servo_yaw.get_angle() / 180 * M_PI;
        position[1] = servo_pitch.get_angle() / 180 * M_PI;
        servo_state_msg.position = position;
        if (pub_servo_state.publish(&servo_state_msg) < 0) {
            terminal.write_string(
                    "node_handler: failed to publish servo_state\n");
        }
    }
}

extern "C" void loop_forever(void) {

    // initialize clock
    MY_DWTTimerInit();

    // initialize debug terminal
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
    
    // initialize bmx055_camera
    terminal.write_string("bmx055_camera initializing...");
    int imu_fail_count = 0;
    while (bmx055_camera.init() < 0) {
        imu_fail_count++;
        terminal.nprintf("failed %d!\n", imu_fail_count);
        HAL_Delay(50);
        terminal.write_string("bmx055_camera initializing...");
    }
    terminal.write_string("ok!\n");
    
    // initialize ros::NodeHandler
    terminal.write_string("ros::NodeHandler initializing...");
    nh.initNode();
    nh.advertise(chatter);
    nh.advertise(pub_imu_camera);
    nh.advertise(pub_temp_camera);
    nh.advertise(pub_mag_camera);
    nh.advertise(pub_servo_state);
    nh.subscribe(receiver);
    terminal.write_string("finished!\n");

    while (1) {
        handle_debug();
        
        update_button();
        handle_button();
        
        update_imu();
        handle_imu();

        update_servo();

        // handle ros::NodeHandler
        nh.spinOnce();
    }

}
