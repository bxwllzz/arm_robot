#ifndef __BMX055_H
#define __BMX055_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "bma2x2.h"
#include "bmg160.h"
#include "bmm050.h"

typedef struct {
    struct bma2x2_t bma2x2;
    struct bmg160_t bmg160;
    struct bmm050_t bmm050;
    struct bma2x2_accel_data_temp raw_acc;
    struct bmg160_data_t raw_gyro;
    struct bmm050_mag_data_s16_t raw_mag;
    double ax, ay, az;
    double gx, gy, gz;
    double mx, my, mz;
} bmx055_t;

int32_t bmx055_init(bmx055_t *pbmx055);
int32_t BMX055_ReadMag(bmx055_t *pbmx055);
int32_t BMX055_ReadAccGyro(bmx055_t *pbmx055);

#ifdef __cplusplus
}
#endif

#endif

