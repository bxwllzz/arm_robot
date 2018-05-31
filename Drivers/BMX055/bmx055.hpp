/*
 * bmx055.hpp
 *
 *  Created on: 2018年5月30日
 *      Author: shuixiang
 */

#pragma once

extern "C" {
#include "bma2x2.h"
#include "bmg160.h"
#include "bmm050.h"

// dev_addr: 7bit
s8 BMX055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s32 iError;
    if (HAL_I2C_Mem_Write(&hi2c1, (dev_addr << 1), reg_addr, 1, reg_data, cnt,
            0xFFFF) == HAL_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    return (s8) iError;
}

s8 BMX055_I2C_bus_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u32 cnt) {
    s32 iError;
    if (HAL_I2C_Mem_Read(&hi2c1, (dev_addr << 1), reg_addr, 1, reg_data, cnt,
            0xFFFF) == HAL_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    return (s8) iError;
}

s8 BMX055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    return BMX055_I2C_bus_burst_read(dev_addr, reg_addr, reg_data, cnt);
}

s8 BMX055_I2C_bus_burst_read2(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt) {
    return BMX055_I2C_bus_burst_read(dev_addr, reg_addr, reg_data, cnt);
}

void BMX055_delay_msec(u32 msec) {
    HAL_Delay(msec);
}

}

class BMX055 {
public:
    struct bma2x2_t bma2x2;
    struct bmg160_t bmg160;
    struct bmm050_t bmm050;

    BMX055(I2C_TypeDef* hi2c, bool PIN_SDO_0 = false, bool PIN_SDO_1 = false,
            bool PIN_CSB_3 = false) {
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
        if (bma2x2_init(&bma2x2) != 0
                || bma2x2.chip_id != 0xFA) {
            com_rslt -= 1;
        }
        RETURN_IF_FAILED;
        if (bmg160_init(&bmg160) != 0
                || bmg160.chip_id != 0x0F) {
            com_rslt -= 1;
        }
        RETURN_IF_FAILED;
        if (bmm050_init(&bmm050) != 0
                || bmm050.company_id != 0x32) {
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
        com_rslt += bma2x2_set_bw(BMA2x2_BW_62_50HZ); // bandwidth: 62.5Hz, update time: 8ms(125Hz)
        RETURN_IF_FAILED;
        com_rslt += bmg160_set_bw(BMG160_BW_32_HZ); // bandwidth: 32Hz, ODR: 100Hz
        RETURN_IF_FAILED;
        com_rslt += bmm050_set_data_rate(BMM050_DATA_RATE_20HZ);    // ODR: 20Hz
        RETURN_IF_FAILED;
        com_rslt += bmm050_set_presetmode(BMM050_PRESETMODE_HIGHACCURACY);
        RETURN_IF_FAILED;
        // 设置量程
        com_rslt += bma2x2_set_range(BMA2x2_RANGE_8G);      // ±8G
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

};
