//
// Created by shuixiang on 2018/10/17.
//

#include "pipe_udp_broadcast.hpp"

#include "debug_terminal.h"

using namespace hustac;

static Pipe* debug_socket = NULL;
static uint8_t debug_socket_snd_buf[512];

extern "C" void debug_socket_init() {
    debug_socket = new PipeUDPBroadcast("0.0.0.0", 1124, 1123, debug_socket_snd_buf, sizeof(debug_socket_snd_buf));
}

extern "C" int debug_socket_write(void* data, size_t len) {
    if (!debug_socket)
        return len;
    InterruptLock lock;
    std::lock_guard<InterruptLock> lock_guard(lock);
    return debug_socket->write(data, len);
}

extern "C" int debug_socket_read(void* data, size_t len) {
    if (!debug_socket)
        return 0;
    InterruptLock lock;
    std::lock_guard<InterruptLock> lock_guard(lock);
    return debug_socket->readsome(data, len);
}

