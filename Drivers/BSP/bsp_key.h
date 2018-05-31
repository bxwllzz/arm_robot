
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __bsp_key_H
#define __bsp_key_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "gpio.h"
 /***************************************
 * 按键按下标置
 * KEY_ON 0
 * KEY_OFF 1
 ***************************************/
#define KEY_ON	0
#define KEY_OFF	1
	
 /***************************************
 * 按键个数
 ***************************************/
#define KEY_NUMBER 5 	
	 
 /***************************************
* 函数功能：检测是否有按键按下
 ***************************************/
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t Down_state);
uint8_t Key_Scan_Clock(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t Down_state);

 /***************************************
 * 函数功能：记录按键被按下次数
 ***************************************/
void Key_Number( GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin ,uint8_t Down_state, uint8_t GPIO_Number);

 /***************************************
 * 函数功能：记录BUF_START按键被按下次数
 ***************************************/
void BUF_START(void);

 /***************************************
 * 函数功能：记录BUF_STOP按键被按下次数
 ***************************************/
void BUF_STOP(void);

 /***************************************
 * 函数功能：记录BUF_STOP1按键被按下次数
 ***************************************/
void BUF_STOP1(void);

 /***************************************
 * 函数功能：记录BUF_STOP2按键被按下次数
 ***************************************/
void BUF_STOP2(void);

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */
