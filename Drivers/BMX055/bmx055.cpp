/*
 * bmx055.cpp
 *
 *  Created on: 2018Äê5ÔÂ30ÈÕ
 *      Author: shuixiang
 */

#include "i2c.h"

#include "bmx055.hpp"

namespace hustac {

BMX055 bmx055_camera(&hi2c1, false, false, false);

}

