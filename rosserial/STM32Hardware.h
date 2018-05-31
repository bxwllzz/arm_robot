/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

#include <cstring>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_dma.h"

#include "usart.h"

class STM32Hardware {
public:
    UART_HandleTypeDef *huart;
protected:
    const static uint16_t rx_buf_len = 512;
    uint8_t rx_buf[rx_buf_len];
    uint32_t rx_read_index;

    const static uint16_t tx_buf_len = 512;
    uint8_t tx_buf[tx_buf_len];
    uint32_t tx_write_index, tx_send_index;

public:
    STM32Hardware() :
            huart(&huart1), rx_read_index(0), tx_write_index(0), tx_send_index(
                    0) {
    }

    void init() {
        _start_dma();
    }

    void _start_dma(void) {
        // reinitialize Rx DMA to set circular mode
        huart->hdmarx->Init.Mode = DMA_CIRCULAR;
        if (HAL_DMA_Init(huart->hdmarx) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }
        // start USART Rx DMA
        if (HAL_UART_Receive_DMA(huart, rx_buf, rx_buf_len) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }
    }

    void _stop_dma(void) {
        if (HAL_UART_DMAStop(huart)) {
            _Error_Handler(__FILE__, __LINE__);
        }
    }

    uint32_t _get_rx_dma_index(void) {
        return rx_buf_len - __HAL_DMA_GET_COUNTER(huart->hdmarx);
    }

    int read() {
        int c = -1;
        if (rx_read_index != _get_rx_dma_index()) {
            c = rx_buf[rx_read_index];
            rx_read_index++;
            rx_read_index %= rx_buf_len;
        }
        return c;
    }

    void _flush(void) {
        static bool mutex = false;

        if ((huart->gState == HAL_UART_STATE_READY) && !mutex) {
            mutex = true;

            if (tx_write_index != tx_send_index) {
                uint16_t len =
                        tx_send_index < tx_write_index ?
                                tx_write_index - tx_send_index :
                                tx_buf_len - tx_send_index;
                if (HAL_UART_Transmit_DMA(huart, &(tx_buf[tx_send_index]), len)
                        != HAL_OK) {
                    _Error_Handler(__FILE__, __LINE__);
                }
                tx_send_index = (tx_send_index + len) % tx_buf_len;
            }
            mutex = false;
        }
    }

    uint32_t _get_available_tx_buf_size() {
        if (huart->gState == HAL_UART_STATE_READY) {
            // dma transmitted
            return tx_buf_len;
        } else {
            // dma transmitting
            uint32_t tx_sending_index = tx_send_index
                    - __HAL_DMA_GET_COUNTER(huart->hdmatx);
            if (tx_sending_index < tx_write_index) {
                return tx_sending_index + tx_buf_len - tx_write_index;
            } else {
                return tx_sending_index - tx_write_index;
            }
        }
    }

    void write(uint8_t* data, int length) {
        int n = length;

        if (n > (int)_get_available_tx_buf_size()) {
            _Error_Handler(__FILE__, __LINE__);
        }

        if (n <= (int)tx_buf_len - tx_write_index) {
            // no need to split data
            memcpy(&(tx_buf[tx_write_index]), data, n);
            tx_write_index = (tx_write_index + n) % tx_buf_len;
        } else {
            // need to split data
            int n_tail = tx_buf_len - tx_write_index;
            memcpy(&(tx_buf[tx_write_index]), data, n_tail);
            tx_write_index = (tx_write_index + n_tail) % tx_buf_len;
            memcpy(tx_buf, &(data[n_tail]), n - n_tail);
            tx_write_index = (tx_write_index + n - n_tail) % tx_buf_len;
        }

        _flush();
    }

    unsigned long time() {
        return HAL_GetTick();
    }

protected:
};

#endif

