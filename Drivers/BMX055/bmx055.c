#include "stm32f1xx_hal.h"
#include "main.h"
#include "i2c.h"

#include "bmx055.h"

#define AX_OFFSET   (-0.005f)
#define AY_OFFSET   (0.02f)
#define AZ_OFFSET   (0.025f)

#define AX_SCALAR   (1.005f)
#define AY_SCALAR   (1.01f)
#define AZ_SCALAR   (1.005f)

#define GX_OFFSET   (-0.002f)
#define GY_OFFSET   (0.086f)
#define GZ_OFFSET   (-0.076f)

#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {-84.0813, -21.3669, -85.0211};
const float magn_ellipsoid_transform[3][3] = {{0.952949, 0.0051534, 0.00712259}, {0.0051534, 0.969072, -0.00789917}, {0.00712259, -0.00789917, 0.997253}};

s8 BMX055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMX055_I2C_bus_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u32 cnt);
s8 BMX055_I2C_bus_burst_read2(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt);
s8 BMX055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BMX055_delay_msec(u32 msec);

int32_t bmx055_init(bmx055_t *pbmx055) {
    
    pbmx055->bma2x2.bus_read = BMX055_I2C_bus_read;
    pbmx055->bma2x2.burst_read = BMX055_I2C_bus_burst_read;
    pbmx055->bma2x2.bus_write = BMX055_I2C_bus_write;
    pbmx055->bma2x2.delay_msec = BMX055_delay_msec;
    pbmx055->bma2x2.dev_addr = BMA2x2_I2C_ADDR1;
    
    pbmx055->bmg160.bus_read = BMX055_I2C_bus_read;
    pbmx055->bmg160.burst_read = BMX055_I2C_bus_burst_read2;
    pbmx055->bmg160.bus_write = BMX055_I2C_bus_write;
    pbmx055->bmg160.delay_msec = BMX055_delay_msec;
    pbmx055->bmg160.dev_addr = BMG160_I2C_ADDR1;
    
    pbmx055->bmm050.bus_read = BMX055_I2C_bus_read;
    pbmx055->bmm050.bus_write = BMX055_I2C_bus_write;
    pbmx055->bmm050.delay_msec = BMX055_delay_msec;
    pbmx055->bmm050.dev_addr = BMM050_I2C_ADDRESS;

#define RETURN_IF_FAILED    if (com_rslt < 0) return com_rslt;
    // 检查chip id
    int32_t com_rslt = 0;
    if (bma2x2_init(&pbmx055->bma2x2) != 0 || pbmx055->bma2x2.chip_id != 0xFA) {
        com_rslt -= 1;
    }
    RETURN_IF_FAILED;
    if (bmg160_init(&pbmx055->bmg160) != 0 || pbmx055->bmg160.chip_id != 0x0F) {
        com_rslt -= 1;
    }
    RETURN_IF_FAILED;
    if (bmm050_init(&pbmx055->bmm050) != 0 || pbmx055->bmm050.company_id != 0x32) {
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
    com_rslt += bma2x2_set_bw(BMA2x2_BW_62_50HZ);   // bandwidth: 62.5Hz, update time: 8ms(125Hz)
    RETURN_IF_FAILED;
    com_rslt += bmg160_set_bw(BMG160_BW_32_HZ);   // bandwidth: 32Hz, ODR: 100Hz
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
    com_rslt += bmm050_write_register(BMM050_SENS_CONTROL_DRDY_EN__REG, &val, 1);
    RETURN_IF_FAILED;
    val = BMM050_INT_CONTROL_DOR_EN__MSK;
    com_rslt += bmm050_write_register(BMM050_INT_CONTROL_DOR_EN__REG, &val, 1);
    RETURN_IF_FAILED;
#undef RETURN_IF_FAILED
    return com_rslt;
}
// 读取磁力计数据并返回数据是否更新
int32_t BMX055_ReadMag(bmx055_t *pbmx055) {
    int32_t com_rslt = 0;
    com_rslt += bmm050_read_mag_data_XYZ(&pbmx055->raw_mag);
    pbmx055->mx = -pbmx055->raw_mag.datay * 0.3;
    pbmx055->my = pbmx055->raw_mag.datax * 0.3;
    pbmx055->mz = pbmx055->raw_mag.dataz * 0.3;
#if CALIBRATION__MAGN_USE_EXTENDED
    double magtemp[3], magtemp2[3];
    magtemp[0] = pbmx055->mx - magn_ellipsoid_center[0];
    magtemp[1] = pbmx055->my - magn_ellipsoid_center[1];
    magtemp[2] = pbmx055->mz - magn_ellipsoid_center[2];
    for(uint8_t x = 0; x < 3; x++) {
        magtemp2[x] = magn_ellipsoid_transform[x][0] * magtemp[0] + magn_ellipsoid_transform[x][1] * magtemp[1] + magn_ellipsoid_transform[x][2] * magtemp[2];
    }
    pbmx055->mx = magtemp2[0];
    pbmx055->my = magtemp2[1];
    pbmx055->mz = magtemp2[2];
#endif
    return com_rslt;
}

int32_t BMX055_ReadAccGyro(bmx055_t *pbmx055) {
    int32_t com_rslt = 0;
    com_rslt += bma2x2_read_accel_xyzt(&pbmx055->raw_acc);
    com_rslt += bmg160_get_data_XYZ(&pbmx055->raw_gyro);
    // 单位转换
    pbmx055->ax = -pbmx055->raw_acc.y / 256.0;
    pbmx055->ay = pbmx055->raw_acc.x / 256.0;
    pbmx055->az = pbmx055->raw_acc.z / 256.0;
    pbmx055->gx = -pbmx055->raw_gyro.datay / 65.6;
    pbmx055->gy = pbmx055->raw_gyro.datax / 65.6;
    pbmx055->gz = pbmx055->raw_gyro.dataz / 65.6;
    // 误差校准
    pbmx055->ax += AX_OFFSET;
    pbmx055->ay += AY_OFFSET;
    pbmx055->az += AZ_OFFSET;
    pbmx055->ax *= AX_SCALAR;
    pbmx055->ay *= AY_SCALAR;
    pbmx055->az *= AZ_SCALAR;
    pbmx055->gx += GX_OFFSET;
    pbmx055->gy += GY_OFFSET;
    pbmx055->gz += GZ_OFFSET;
    return com_rslt;
}


// dev_addr: 7bit
s8 BMX055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 iError;
    if (HAL_I2C_Mem_Write(&hi2c1, (dev_addr << 1), reg_addr, 1, reg_data, cnt, 0xFFFF) == HAL_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    return (s8)iError;
}

s8 BMX055_I2C_bus_burst_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u32 cnt) {
    s32 iError;
    if (HAL_I2C_Mem_Read(&hi2c1, (dev_addr << 1), reg_addr, 1, reg_data, cnt, 0xFFFF) == HAL_OK) {
        iError = 0;
    } else {
        iError = -1;
    }
    return (s8)iError;
}

s8 BMX055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    return BMX055_I2C_bus_burst_read(dev_addr, reg_addr, reg_data, cnt);
}

s8 BMX055_I2C_bus_burst_read2(u8 dev_addr, u8 reg_addr, u8 *reg_data, s32 cnt) {
    return BMX055_I2C_bus_burst_read(dev_addr, reg_addr, reg_data, cnt);
}

void BMX055_delay_msec(u32 msec)
{
    HAL_Delay(msec);
}




