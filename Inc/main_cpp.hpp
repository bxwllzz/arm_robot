#pragma once

#include <ros.h>

#include "dmabuffer_uart.hpp"

namespace hustac {

extern DMABuffer_UART<512, 128> terminal;

extern ros::NodeHandle nh;
    
}
