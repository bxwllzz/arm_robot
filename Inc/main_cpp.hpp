#pragma once

#ifndef M_PI
#define M_PI (3.14159265358979323846264338327950288)
#endif

#include <ros.h>

#include "dmabuffer_uart.hpp"

namespace hustac {

extern DMABuffer_UART<512, 128> terminal;

extern ros::NodeHandle nh;
    
}
