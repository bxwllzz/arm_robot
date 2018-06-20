/*
 * OutputBufferUARTDMA.hpp
 *
 *  Created on: 2018年5月31日
 *      Author: shuixiang
 */

#pragma once

#include <cstring>
#include <cstdlib>
#include <cstdarg>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

namespace hustac {

template <int tx_buf_len, int rx_buf_len>
class DMABuffer_UART {
private:
    UART_HandleTypeDef *huart;

    uint8_t tx_buf[tx_buf_len];
    volatile bool tx_is_writing = false;
    size_t tx_write_index = 0;
    size_t tx_need_send = 0;

    uint8_t rx_buf[rx_buf_len];
    size_t rx_read_index = 0;
public:
    DMABuffer_UART(UART_HandleTypeDef* _huart) :
            huart(_huart) {
    }

    int init() {
        tx_write_index = 0;
        rx_read_index = 0;

        // interrupt of both UART and DMA should be enabled
        // Tx DMA should be normal mode
        // Rx DMA should be circular mode
        // start UART Rx DMA
        if (HAL_UART_Receive_DMA(huart, rx_buf, (uint16_t) rx_buf_len)
                != HAL_OK) {
            _Error_Handler((char*)__FILE__, __LINE__);
        }
        return 0;
    }

    ~DMABuffer_UART() {
        if (HAL_UART_DMAStop(huart)) {
            _Error_Handler((char*)__FILE__, __LINE__);
        }
    }

    /* 读取部分 */

    // 获取当前DMA读取索引
    size_t _get_rx_dma_index() {
        return rx_buf_len - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    }

    // 可读取的字节数
    int get_rx_available() {
//        if (huart->RxState == HAL_UART_STATE_READY) {
//            if (HAL_UART_Receive_DMA(huart, rx_buf, (uint16_t) rx_buf_len)
//                != HAL_OK) {
//                _Error_Handler((char*)__FILE__, __LINE__);
//            }
//        }
        size_t rx_dma_index = _get_rx_dma_index();
        if (rx_dma_index >= rx_read_index) {
            return rx_dma_index - rx_read_index;
        } else {
            return rx_dma_index + rx_buf_len - rx_read_index;
        }
    }

    // 读取一个字节
    // 出错时返回
    int get() {
        if (get_rx_available() > 0) {
            uint8_t ch = rx_buf[rx_read_index];
            rx_read_index++;
            rx_read_index %= rx_buf_len;
            return ch;
        } else {
            return -1;
        }
    }

    // 读取但是不取走一个字符, 无范围检查
    uint8_t _peek_unsafe(size_t index = 0) {
        return rx_buf[index % rx_buf_len];
    }

    // 读取(但不取走)一个字符
    int peek(size_t index = 0) {
        if (get_rx_available() > (int)index) {
            return _peek_unsafe(index);
        } else {
            return -1;
        }
    }

    // 读取一块数据
    int read(uint8_t* data, size_t count) {
        if (get_rx_available() >= (int)count) {
            if (rx_read_index + count < rx_buf_len) {
                memcpy(data, &rx_buf[rx_read_index], count);
            } else {
                memcpy(data, &rx_buf[rx_read_index],
                        rx_buf_len - rx_read_index);
                memcpy(data + rx_buf_len - rx_read_index, &rx_buf[0],
                        rx_read_index + count - rx_buf_len);
            }
            rx_read_index += count;
            rx_read_index %= rx_buf_len;
            return count;
        } else {
            return -1;
        }
    }

    // 读取所有可用的数据
    int readsome(uint8_t* data, size_t max_len) {
        int count = get_rx_available();
        if (count <= 0) {
            return 0;
        }
        if (rx_read_index + count < rx_buf_len) {
            std::memcpy(data, &rx_buf[rx_read_index], count);
        } else {
            std::memcpy(data, &rx_buf[rx_read_index],
                    rx_buf_len - rx_read_index);
            std::memcpy(data + rx_buf_len - rx_read_index, &rx_buf[0],
                    rx_read_index + count - rx_buf_len);
        }
        rx_read_index += count;
        rx_read_index %= rx_buf_len;
        return count;
    }

    /* 写入部分, 允许在中断中使用 */

#define ENTER_CRITICAL()    uint32_t prim = __get_PRIMASK(); __disable_irq()
#define REENTER_CRITICAL()  prim = __get_PRIMASK(); __disable_irq()
#define EXIT_CRITICAL()     __set_PRIMASK(prim)

    void on_uart_error(UART_HandleTypeDef* _huart) {
        if (_huart == huart) {
            rx_read_index = 0;
            if (HAL_UART_Receive_DMA(huart, rx_buf, (uint16_t) rx_buf_len)
                    != HAL_OK) {
                _Error_Handler((char*)__FILE__, __LINE__);
            }
        }
    }
    
    void on_tx_dma_complete(UART_HandleTypeDef* _huart) {
        if (_huart == huart) {
            _flush();
        }
    }

    void _flush() {
        ENTER_CRITICAL();
        if (!tx_is_writing && huart->gState == HAL_UART_STATE_READY && tx_need_send > 0) {
            if (tx_need_send > 0) {
                size_t block_begin_index;
                size_t block_len;
                if (tx_need_send <= tx_write_index) {
                    block_begin_index = tx_write_index - tx_need_send;
                    block_len = tx_need_send;
                } else {
                    block_begin_index = tx_write_index + tx_buf_len - tx_need_send;
                    block_len = tx_buf_len - block_begin_index;
                }
                if (HAL_UART_Transmit_DMA(huart, &(tx_buf[block_begin_index]), block_len)
                        == HAL_OK) {
                    tx_need_send -= block_len;
                }
            }
        }
        EXIT_CRITICAL();
    }

    // 获取当前可用的输出缓冲区尺寸
    int get_tx_available() {
        ENTER_CRITICAL();
        size_t tx_dma_remain = __HAL_DMA_GET_COUNTER(huart->hdmatx);
        int tx_available = tx_buf_len - tx_need_send - tx_dma_remain;
        EXIT_CRITICAL();
        return tx_available;
    }

    // 写入一块数据
    int write(const uint8_t* data, size_t count) {
        int ret;
        ENTER_CRITICAL();
        size_t tx_dma_remain = __HAL_DMA_GET_COUNTER(huart->hdmatx);
        if (tx_buf_len - tx_need_send - tx_dma_remain >= count) {
            size_t begin_index = tx_write_index;
            tx_need_send += count;
            tx_write_index += count;
            tx_write_index %= tx_buf_len;
            EXIT_CRITICAL();

            bool writing_status = tx_is_writing;
            tx_is_writing = true;
            if (begin_index + count < tx_buf_len) {
                std::memcpy(&tx_buf[begin_index], data, count);
            } else {
                std::memcpy(&tx_buf[begin_index], data,
                        tx_buf_len - begin_index);
                std::memcpy(&tx_buf[0], data + tx_buf_len - begin_index,
                        begin_index + count - tx_buf_len);
            }

            tx_is_writing = writing_status;

            _flush();

            ret = count;
        } else {
            EXIT_CRITICAL();
            ret = -1;
        }
        return ret;
    }

    int write_string(const char* str) {
        return write((const uint8_t*)str, std::strlen(str));
    }

    // 返回实际输出的字符数, 失败则返回0
    template <int max_len=20>
    int nprintf(const char* fmt...) {
        uint8_t buf[max_len];
        std::va_list args;
        va_start(args, fmt); //获得可变参数列表
        int n = vsnprintf((char*)buf, max_len, fmt, args);
        va_end(args);//释放资源
        int ret = write(buf, n);
        return ret;
    }

#undef ENTER_CRITICAL
#undef REENTER_CRITICAL
#undef EXIT_CRITICAL
};

}
