/*
 * bmx055.hpp
 *
 *  Created on: 2018年5月30日
 *      Author: shuixiang
 */

#pragma once

#include <inttypes.h>
#include <limits>

#include "stm32f1xx_hal_i2c.h"

#include "main_cpp.hpp"
#include "high_resolution_clock.h"

extern "C" {
#include "bma2x2.h"
#include "bmg160.h"
#include "bmm050.h"

// dev_addr: 7bit
static s8 BMX055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s32 iError;
    if (HAL_I2C_Mem_Write(&hi2c1, (dev_addr << 1), reg_addr, 1, reg_data, cnt,
            0xFFFF) == HAL_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    return (s8) iError;
}

static s8 BMX055_I2C_bus_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u32 cnt) {
    s32 iError;
    if (HAL_I2C_Mem_Read(&hi2c1, (dev_addr << 1), reg_addr, 1, reg_data, cnt,
            0xFFFF) == HAL_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    return (s8) iError;
}

static s8 BMX055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    return BMX055_I2C_bus_burst_read(dev_addr, reg_addr, reg_data, cnt);
}

static s8 BMX055_I2C_bus_burst_read2(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt) {
    return BMX055_I2C_bus_burst_read(dev_addr, reg_addr, reg_data, cnt);
}

static void BMX055_delay_msec(u32 msec) {
    HAL_Delay(msec);
}

}

namespace hustac {

struct IMUMeasure {
    uint64_t nsec = 0;
    float accel[3] = { std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN() };
    float gyro[3] = { std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN() };
    float temperature = std::numeric_limits<float>::signaling_NaN();
};

struct MagMeasure {
    uint64_t nsec = 0;
    float mag[3] = { std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN(), std::numeric_limits<float>::signaling_NaN() };
};

class BMX055 {
public:
    I2C_HandleTypeDef* hi2c;

    struct bma2x2_t bma2x2;
    struct bmg160_t bmg160;
    struct bmm050_t bmm050;

    enum class State {
        IDLE, READ_ACCEL, WAIT_ACCEL, READ_GYRO, WAIT_GYRO, READ_MAG, WAIT_MAG,
    };
    volatile State state = State::IDLE;
    volatile bool _FSM_lock = false;
    volatile bool _FSM_need_update = false;
    
    volatile bool i2c_dma_read_complete = true;
    volatile bool i2c_dma_read_failed = false;

    volatile uint32_t next_read_imu = 0;    // ms
    const uint32_t imu_update_interval = 1; // ms

    volatile uint32_t next_read_mag = 0;    // ms
    const uint32_t mag_update_interval = 50;    // ms

    volatile uint32_t count_imu_measure = 0;
    volatile uint32_t count_mag_measure = 0;

private:
    volatile uint64_t last_read_accel = 0;   // ns
    volatile uint64_t last_read_gyro = 0;    // ns
    volatile uint64_t last_read_mag = 0;     // ns

    // raw data
    struct bma2x2_accel_data_temp raw_acc;
    struct bmg160_data_t raw_gyro;
    struct bmm050_mag_data_s16_t raw_mag;

    // output data
    IMUMeasure imu_measure;
    bool is_new_imu_measure = false;
    MagMeasure mag_measure;
    bool is_new_mag_measure = false;

public:
    BMX055(I2C_HandleTypeDef* _hi2c, bool PIN_SDO_0 = false, bool PIN_SDO_1 =
            false, bool PIN_CSB_3 = false) :
            hi2c(_hi2c) {
        bma2x2.bus_read = BMX055_I2C_bus_read;
        bma2x2.burst_read = BMX055_I2C_bus_burst_read;
        bma2x2.bus_write = BMX055_I2C_bus_write;
        bma2x2.delay_msec = BMX055_delay_msec;

        bmg160.bus_read = BMX055_I2C_bus_read;
        bmg160.burst_read = BMX055_I2C_bus_burst_read2;
        bmg160.bus_write = BMX055_I2C_bus_write;
        bmg160.delay_msec = BMX055_delay_msec;

        bmm050.bus_read = BMX055_I2C_bus_read;
        bmm050.bus_write = BMX055_I2C_bus_write;
        bmm050.delay_msec = BMX055_delay_msec;

        // i2c addr
        if (!PIN_SDO_0) {
            bma2x2.dev_addr = BMA2x2_I2C_ADDR1;
        } else {
            bma2x2.dev_addr = BMA2x2_I2C_ADDR2;
        }
        if (!PIN_SDO_1) {
            bmg160.dev_addr = BMG160_I2C_ADDR1;
        } else {
            bmg160.dev_addr = BMG160_I2C_ADDR2;
        }
        if (!PIN_CSB_3 && !PIN_SDO_0) {
            bmm050.dev_addr = BMM050_I2C_ADDRESS;
        } else if (!PIN_CSB_3 && PIN_SDO_0) {
            bmm050.dev_addr = BMM050_I2C_ADDRESS + 1;
        } else if (PIN_CSB_3 && !PIN_SDO_0) {
            bmm050.dev_addr = BMM050_I2C_ADDRESS + 2;
        } else {
            bmm050.dev_addr = BMM050_I2C_ADDRESS + 3;
        }
    }

    int init() {
#define RETURN_IF_FAILED    if (com_rslt < 0) return com_rslt;
        // 检查chip id
        int com_rslt = 0;
        if (bma2x2_init(&bma2x2) != 0 || bma2x2.chip_id != 0xFA) {
            com_rslt -= 1;
        }
        RETURN_IF_FAILED;
        if (bmg160_init(&bmg160) != 0 || bmg160.chip_id != 0x0F) {
            com_rslt -= 1;
        }
        RETURN_IF_FAILED;
        if (bmm050_init(&bmm050) != 0 || bmm050.company_id != 0x32) {
            com_rslt -= 1;
        }
        RETURN_IF_FAILED;
        // 设置电源为工作模式
        com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
        RETURN_IF_FAILED;
        com_rslt += bmg160_set_power_mode(BMG160_MODE_NORMAL);
        RETURN_IF_FAILED;
        com_rslt += bmm050_set_power_mode(1);
        RETURN_IF_FAILED;
        com_rslt += bmm050_set_functional_state(BMM050_NORMAL_MODE);
        RETURN_IF_FAILED;
        // 设置带宽/输出频率
        // 由于I2C总线带宽400kHz限制, 并保证加速度计与陀螺仪准确同步, 加速度计与陀螺仪的数据采样频率设定为 1kHz
        // 理论带宽占用率约为 1k*(7+3+6+3)*10/400k = 47.5%
        com_rslt += bma2x2_set_bw(BMA2x2_BW_500HZ); // 加速度计带宽 500Hz, 对应数据采样频率为 1kHz
        RETURN_IF_FAILED;
        com_rslt += bmg160_set_bw(BMG160_BW_116_HZ); // 陀螺仪带宽 116Hz, 对应数据采样频率ODR为 1kHz
        RETURN_IF_FAILED;
        com_rslt += bmm050_set_data_rate(BMM050_DATA_RATE_20HZ); // 磁力计数据采样ODR: 20Hz
        RETURN_IF_FAILED;
        com_rslt += bmm050_set_presetmode(BMM050_PRESETMODE_HIGHACCURACY);
        RETURN_IF_FAILED;
        // 设置量程
        com_rslt += bma2x2_set_range(BMA2x2_RANGE_4G);      // ±4G
        RETURN_IF_FAILED;
        com_rslt += bmg160_set_range_reg(BMG160_RANGE_500); // ±500°/s
        RETURN_IF_FAILED;
        // 设置数据就绪中断(I2C轮询中断标志位)
//    com_rslt += bma2x2_set_new_data(BMA2x2_INTR1_NEWDATA, INTR_ENABLE);
//    com_rslt += bma2x2_set_intr_enable(BMA2x2_DATA_ENABLE, INTR_ENABLE);
        com_rslt += bmg160_set_data_enable(BMG160_ENABLE);
        RETURN_IF_FAILED;
        com_rslt += bmg160_set_intr_data(BMG160_INTR1_DATA, BMG160_ENABLE);
        RETURN_IF_FAILED;
        uint8_t val;
        val = BMM050_SENS_CONTROL_DRDY_EN__MSK;
        com_rslt += bmm050_write_register(BMM050_SENS_CONTROL_DRDY_EN__REG,
                &val, 1);
        RETURN_IF_FAILED;
        val = BMM050_INT_CONTROL_DOR_EN__MSK;
        com_rslt += bmm050_write_register(BMM050_INT_CONTROL_DOR_EN__REG, &val,
                1);
        RETURN_IF_FAILED;

#undef RETURN_IF_FAILED
        return com_rslt;
    }

    // 处理获得的加速度计陀螺仪数据
    void _process_imu() {
        imu_measure.accel[0] = raw_acc.x;
        imu_measure.accel[1] = raw_acc.y;
        imu_measure.accel[2] = raw_acc.z;
        imu_measure.temperature = raw_acc.temp;

        // todo: 单位转换
//        ax = -raw_acc.y / 256.0;
//        ay = raw_acc.x / 256.0;
//        az = raw_acc.z / 256.0;
//        // 误差校准
//        ax += AX_OFFSET;
//        ay += AY_OFFSET;
//        az += AZ_OFFSET;
//        ax *= AX_SCALAR;
//        ay *= AY_SCALAR;
//        az *= AZ_SCALAR;

        imu_measure.gyro[0] = raw_gyro.datax;
        imu_measure.gyro[1] = raw_gyro.datay;
        imu_measure.gyro[2] = raw_gyro.dataz;

        // todo: 单位转换
//        gx = -raw_gyro.datay / 65.6;
//        gy = raw_gyro.datax / 65.6;
//        gz = raw_gyro.dataz / 65.6;
//        // 误差校准
//        gx += GX_OFFSET;
//        gy += GY_OFFSET;
//        gz += GZ_OFFSET;
    }

    // 处理获取的磁力计数据
    void _process_mag() {
        mag_measure.mag[0] = raw_mag.datax;
        mag_measure.mag[1] = raw_mag.datay;
        mag_measure.mag[2] = raw_mag.dataz;

        // todo: 单位转换
//        mx = -raw_mag.datay * 0.3;
//        my = raw_mag.datax * 0.3;
//        mz = raw_mag.dataz * 0.3;

    }

    int _FSM_update() {
        int com_rslt = 0;
        bool stop_loop = false;
        while (!stop_loop) {
            switch (state) {
            case State::IDLE:
                if (HAL_GetTick() >= next_read_imu) {
                    state = State::READ_ACCEL;
                } else if (HAL_GetTick() >= next_read_mag) {
                    state = State::READ_MAG;
                } else {
                    // wait until next imu read
                    stop_loop = true;
                }
                break;
            case State::READ_ACCEL:
                if (HAL_I2C_Mem_Read_DMA(hi2c, (bma2x2.dev_addr << 1),
                BMA2x2_ACCEL_X12_LSB_REG, 1, (uint8_t*) &raw_acc,
                BMA2x2_ACCEL_XYZ_TEMP_DATA_SIZE) == HAL_OK) {
                    uint64_t now = MY_GetNanoSecFromCycle(
                            MY_GetCycleCount());
                    if ((now - last_read_accel) / 1000 >= imu_update_interval * 1300) {
                        terminal.nprintf<50>("bmx055: read accel, bad interval %" PRIu32 " us\n", (uint32_t)(now - last_read_accel) / 1000);
                    }
                    next_read_imu += imu_update_interval;
                    
                    state = State::WAIT_ACCEL;
                    i2c_dma_read_complete = false;
                    i2c_dma_read_failed = false;
                    last_read_accel = now;
                } else {
                    // cannot start dma read
                    stop_loop = true;
                    com_rslt = -1;
                    terminal.write_string("bmx055: read accel, cannot start i2c dma read\n");
                }
                break;
            case State::WAIT_ACCEL:
                if (i2c_dma_read_failed) {
                    terminal.write_string("bmx055: read accel error, retry\n");
                    state = State::READ_ACCEL;
                } else if (i2c_dma_read_complete) {
//                    terminal.write_string("bmx055: read accel complete\n");
                    
                    state = State::READ_GYRO;
                } else {
                    // wait until dma read finished
                    stop_loop = true;
                }
                break;
            case State::READ_GYRO:
                if (HAL_I2C_Mem_Read_DMA(hi2c, (bmg160.dev_addr << 1),
                BMG160_RATE_X_LSB_BIT__REG, 1, (uint8_t*) &raw_gyro,
                BMG160_ALL_DATA_FRAME_LENGTH) == HAL_OK) {
                    uint64_t now = MY_GetNanoSecFromCycle(
                            MY_GetCycleCount());
                    if ((now - last_read_gyro) / 1000 >= imu_update_interval * 1300) {
                        terminal.nprintf<50>("bmx055: read gyro, bad interval %" PRIu32 " us\n", (uint32_t)(now - last_read_gyro) / 1000);
                    }
                    state = State::WAIT_GYRO;
                    i2c_dma_read_complete = false;
                    i2c_dma_read_failed = false;
                    last_read_gyro = now;
                } else {
                    // cannot start dma read
                    stop_loop = true;
                    com_rslt = -1;
                    terminal.write_string("bmx055: read gyro, cannot start i2c dma read\n");
                }
                break;
            case State::WAIT_GYRO:
                if (i2c_dma_read_failed) {
                    terminal.write_string("bmx055: read gyro error, retry\n");
                    state = State::READ_GYRO;
                } else  if (i2c_dma_read_complete) {
//                    terminal.write_string("bmx055: read gyro complete\n");
                    
                    if (is_new_imu_measure) {
                        terminal.write_string("bmx055: imu_meaure overwrite, please get it in time!\n");
                    }
                    _process_imu();
                    imu_measure.nsec = (last_read_accel + last_read_gyro) / 2;
                    is_new_imu_measure = true;
                    count_imu_measure++;
                    
                    state = State::IDLE;
                } else {
                    // wait until dma read finished
                    stop_loop = true;
                }
                break;
            case State::READ_MAG:
                if (HAL_I2C_Mem_Read_DMA(hi2c, (bmm050.dev_addr << 1),
                BMM050_DATA_X_LSB, 1, (uint8_t*) &raw_mag,
                BMM050_ALL_DATA_FRAME_LENGTH) == HAL_OK) {
                    uint64_t now = MY_GetNanoSecFromCycle(
                            MY_GetCycleCount());
                    if ((now - last_read_mag) / 1000 >= mag_update_interval * 1300) {
                        terminal.nprintf<50>("bmx055: read mag, bad interval %" PRIu32 " us\n", (uint32_t)(now - last_read_mag) / 1000);
                    }
                    next_read_mag += mag_update_interval;
                    
                    state = State::WAIT_MAG;
                    i2c_dma_read_complete = false;
                    i2c_dma_read_failed = false;
                    last_read_mag = now;
                } else {
                    // cannot start dma read
                    com_rslt = -1;
                    terminal.write_string("bmx055: read mag, cannot start i2c dma read\n");
                    
                    state = State::IDLE;
                }
                break;
            case State::WAIT_MAG:
                if (i2c_dma_read_failed) {
                    terminal.write_string("bmx055: read mag error, retry\n");
                    com_rslt = -1;
                    
                    state = State::IDLE;
                } else if (i2c_dma_read_complete) {
//                    terminal.write_string("bmx055: read mag complete\n");
                    
                    if (is_new_mag_measure) {
                        terminal.write_string("bmx055: mag_measure overwrite, please get it in time!\n");
                    }
                    _process_mag();
                    mag_measure.nsec = last_read_mag;
                    is_new_mag_measure = true;
                    count_mag_measure++;

                    state = State::IDLE;
                } else {
                    // wait until dma read finished
                    stop_loop = true;
                }
                break;
            }
        }
        return com_rslt;
    }

    // should be called when i2c dma mem read complete
    void on_i2c_dma_complete() {
        i2c_dma_read_complete = true;
        if (!_FSM_lock) {
            _FSM_lock = true;
            _FSM_update();
            _FSM_lock = false;
        } else {
            if (_FSM_need_update) {
                terminal.write_string("bmx055: failed to lock _FSM_lock in interrupt\n");
            }
            _FSM_need_update = true;
        }
    }
    
    // should be called when i2c dma mem read failed
    void on_i2c_dma_failed() {
        i2c_dma_read_failed = true;
        if (!_FSM_lock) {
            _FSM_lock = true;
            _FSM_update();
            _FSM_lock = false;
        } else {
            if (_FSM_need_update) {
                terminal.write_string("bmx055: failed to lock _FSM_lock in interrupt\n");
            }
            _FSM_need_update = true;
        }
    }

    // should only be called in main loop
    int update() {
        int ret;
        do {
            _FSM_need_update = false;
            _FSM_lock = true;
            ret = _FSM_update();
            _FSM_lock = false;
        } while (_FSM_need_update);
        return ret;
    }

    // should only be called in main loop
    int get_imu_measure(IMUMeasure& measure) {
        int ret;
        _FSM_lock = true;
        if (is_new_imu_measure) {
            is_new_imu_measure = false;
            measure = imu_measure;
            ret = 1;
        } else {
            ret = 0;
        }
        _FSM_lock = false;
        return ret;
    }

    // should only be called in main loop
    int get_mag_measure(MagMeasure& measure) {
        int ret;
        _FSM_lock = true;
        if (is_new_mag_measure) {
            is_new_mag_measure = false;
            measure = mag_measure;
            ret = 1;
        } else {
            ret = 0;
        }
        _FSM_lock = false;
        return ret;
    }
};

extern BMX055 bmx055_camera;

}
