//
// Created by shuixiang on 2018/10/17.
//

#ifndef ARM_ROBOT_F427_V3_0_PIPE_UDP_BROADCAST_HPP
#define ARM_ROBOT_F427_V3_0_PIPE_UDP_BROADCAST_HPP

#include <cstring>

#include "lwip/sockets.h"
#include "mutex.hpp"

#include "pipe.hpp"

namespace hustac {

class PipeUDPBroadcast : public Pipe {
public:
    PipeUDPBroadcast(const char *bind_ip, uint16_t recv_port, uint16_t send_port, uint8_t* snd_buf, size_t snd_buf_len);
    bool connected() override { return sock_in_fd_ >= 0 && sock_out_fd_ >= 0; }
    int flush();
    int readsome(void *, size_t) override;
    int write(void *, size_t) override;
    ~PipeUDPBroadcast() override;
protected:
    void _reconnect();
    void _spin_once();
protected:
    struct sockaddr_in bind_addr_;
    struct sockaddr_in send_addr_;
    int sock_in_fd_ = -1;
    int sock_out_fd_ = -1;

    uint8_t* const snd_buf_ = nullptr;
    const size_t snd_buf_len_ = 0;
    size_t snd_buf_index_in_ = 0;
    size_t snd_buf_index_out_ = 0;
    
    const uint32_t flush_timeout_ = 100;    // 刷新输出缓存超时
    uint32_t t_flush_ = 0;                  // 上一次刷新输出缓存
    
};

inline PipeUDPBroadcast::PipeUDPBroadcast(const char* bind_ip, uint16_t recv_port, uint16_t send_port, uint8_t* snd_buf,
                                          size_t snd_buf_len) :
    snd_buf_(snd_buf), snd_buf_len_(snd_buf_len) {
    memset(&bind_addr_, 0, sizeof(bind_addr_));
    bind_addr_.sin_family = AF_INET;
    bind_addr_.sin_addr.s_addr = inet_addr(bind_ip);
    bind_addr_.sin_port = htons(recv_port);
    memset(&send_addr_, 0, sizeof(send_addr_));
    send_addr_.sin_family = AF_INET;
    send_addr_.sin_addr.s_addr = inet_addr("255.255.255.255");
    send_addr_.sin_port = htons(send_port);
    _spin_once();
}

inline PipeUDPBroadcast::~PipeUDPBroadcast() {
    if (sock_in_fd_ >= 0)
        lwip_close(sock_in_fd_);
    sock_in_fd_ = -1;
    if (sock_out_fd_ >= 0)
        lwip_close(sock_out_fd_);
    sock_out_fd_ = -1;
}

inline void PipeUDPBroadcast::_reconnect() {
    if (sock_in_fd_ >= 0)
        lwip_close(sock_in_fd_);
    sock_in_fd_ = -1;
    if (sock_out_fd_ >= 0)
        lwip_close(sock_out_fd_);
    sock_out_fd_ = -1;
    _spin_once();
}

inline void PipeUDPBroadcast::_spin_once() {
    if (sock_in_fd_ < 0) {
        int sock_fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_fd < 0) {
            perror("PipeUDPBroadcast: Can not create socket");
            return;
        }
        
        // enable nonblocking
        int flags;
        flags = lwip_fcntl(sock_fd, F_GETFL, 0);
        if (flags < 0) {
            perror("PipeUDPBroadcast: lwip_fcntl(F_GETFL) failed");
            lwip_close(sock_fd);
            return;
        }
        flags |= O_NONBLOCK;
        if (lwip_fcntl(sock_fd, F_SETFL, flags) < 0) {
            perror("PipeUDPBroadcast: lwip_fcntl(F_SETFL, O_NONBLOCK) failed");
            lwip_close(sock_fd);
            return;
        }
        
        if (lwip_bind(sock_fd, (sockaddr *) &bind_addr_, (socklen_t) sizeof(bind_addr_)) < 0) {
            perror("PipeUDPBroadcast: lwip_bind() failed");
            lwip_close(sock_fd);
            return;
        }
        
        sock_in_fd_ = sock_fd;
    }
    if (sock_out_fd_ < 0) {
        int sock_fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock_fd < 0) {
            perror("PipeUDPBroadcast: Can not create socket");
            return;
        }
        
        // enable broadcast
        int optval = 1;
        if (lwip_setsockopt(sock_fd, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval)) < 0) {
            perror("PipeUDPBroadcast: lwip_setsockopt(SO_BROADCAST, 1) failed");
            lwip_close(sock_fd);
            return;
        }
        
        // enable nonblocking
        int flags;
        flags = lwip_fcntl(sock_fd, F_GETFL, 0);
        if (flags < 0) {
            perror("PipeUDPBroadcast: lwip_fcntl(F_GETFL) failed");
            lwip_close(sock_fd);
            return;
        }
        flags |= O_NONBLOCK;
        if (lwip_fcntl(sock_fd, F_SETFL, flags) < 0) {
            perror("PipeUDPBroadcast: lwip_fcntl(F_SETFL, O_NONBLOCK) failed");
            lwip_close(sock_fd);
            return;
        }
        
        if (lwip_connect(sock_fd, (sockaddr *) &send_addr_, (socklen_t) sizeof(send_addr_)) < 0) {
            perror("PipeUDPBroadcast: lwip_connect() failed");
            lwip_close(sock_fd);
            return;
        }
    
        sock_out_fd_ = sock_fd;
        snd_buf_index_in_ = 0;
        snd_buf_index_out_ = 0;
        t_flush_ = 0;
    }
}

inline int PipeUDPBroadcast::readsome(void *data, size_t len) {
    if (sock_in_fd_ >= 0) {
        int ret = lwip_read(sock_in_fd_, data, len);
        if (ret >= 0) {
            // 成功读取
            return ret;
        } else {
            // 读取失败, 重新连接
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("PipeUDPBroadcast: lwip_read() failed");
                _reconnect();
                return -2;
            }
            return -1;
        }
    } else {
        _spin_once();
        return -3;
    }
}

inline int PipeUDPBroadcast::flush() {
    if (snd_buf_index_in_ <= snd_buf_index_out_) {
        return 0;
    }
    if (sock_out_fd_ >= 0) {
        int bytes_sent = 0;
        while (snd_buf_index_in_ > snd_buf_index_out_) {
            void* data = snd_buf_ + snd_buf_index_out_;
            size_t len = snd_buf_index_in_ - snd_buf_index_out_;
            int ret = lwip_write(sock_out_fd_, data, len);
            if (ret == len) {
                // 成功写入
                snd_buf_index_out_ += ret;
                bytes_sent += ret;
            } else if (ret >= 0) {
                // 仅成功写入部分
                fprintf(stderr, "PipeUDPBroadcast: lwip_write() %d/%d\n", ret, len);
                snd_buf_index_out_ += ret;
                bytes_sent += ret;
                break;
            } else {
                // 写入失败, 重新连接
                perror("PipeUDPBroadcast: lwip_write() failed");
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    _reconnect();
                }
                break;
            }
        }
        if (snd_buf_index_in_ == snd_buf_index_out_) {
            snd_buf_index_in_ = 0;
            snd_buf_index_out_ = 0;
        }
        if (bytes_sent) {
            t_flush_ = HAL_GetTick();
        }
        return bytes_sent;
    } else {
        _spin_once();
        return -3;
    }
}

inline int PipeUDPBroadcast::write(void *data, size_t len) {
    int ret = 0;
    if (len <= snd_buf_len_ - snd_buf_index_in_) {
        // 缓冲区空间足够
        std::memcpy(snd_buf_ + snd_buf_index_in_, data, len);
        snd_buf_index_in_ += len;
        ret = len;
    } else {
        // 缓冲区空间不足, 刷新缓冲区然后重试
        flush();
        if (len <= snd_buf_len_ - snd_buf_index_in_) {
            // 缓冲区空间足够
            std::memcpy(snd_buf_ + snd_buf_index_in_, data, len);
            snd_buf_index_in_ += len;
            ret = len;
        } else {
            // 缓冲区空间依旧不足, 写入部分数据
            std::memcpy(snd_buf_ + snd_buf_index_in_, data, snd_buf_len_ - snd_buf_index_in_);
            ret = snd_buf_len_ - snd_buf_index_in_;
            snd_buf_index_in_ = snd_buf_len_;
        }
    }
    
    if (std::memchr(data, '\n', len) != nullptr) {
        // 输出换行, 则刷新输出缓冲
        flush();
    }
    if (HAL_GetTick() - t_flush_ >= flush_timeout_) {
        // 超时, 强制刷新
        flush();
    }
    return ret;
}

}

#endif //ARM_ROBOT_F427_V3_0_PIPE_UDP_BROADCAST_HPP
