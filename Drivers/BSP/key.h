
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __key_H
#define __key_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "gpio.h"

/* 定义一个枚举类型列出该系统所有状态 */
typedef enum
{
    NoKeyDown = 0,
    KeySureDown ,
    OnceKeyDown,
    KeySureRelease
}StateStatus;


typedef enum
{
    ContinueNoKeyDown = 0,
    ContinueKeySureDown ,
    ContinueKeyDown,
    ContinueKeySureRelease
}Continue_StateStatus;

typedef enum
{
    Continue_No_Key_Down = 0,
    Continue_Key_Sure_Down ,
    Continue_Key_Down,
}Continue_State_Down_Status;

typedef enum
{
    Continue_No_Key_Release = 0,
    Continue_Key_Sure_Release ,
    Continue_Key_Release,
}Continue_State_Release_Status;
    

struct KeyStateStatus
{
	StateStatus Start_Status;
	StateStatus Stop_Status;
	Continue_StateStatus Stop1_Status;
	Continue_StateStatus Stop2_Status;
};

struct Continue_Key_Down_StateStatus
{
	Continue_State_Down_Status Stop_one_Status;
	Continue_State_Down_Status Stop_two_Status;
};

struct Continue_Key_Release_StateStatus
{
	Continue_State_Release_Status Stop_one_Status;
	Continue_State_Release_Status Stop_two_Status;
};

extern uint32_t HUST_BUTTOM_START_STATUS_COUNT_ONE ; //BUF_START按键次数
extern uint32_t HUST_BUTTOM_STOP_STATUS_COUNT_ONE  ; //BUF_STOP按键次数
extern uint32_t HUST_BUTTOM_STOP1_STATUS_COUNT_ONE ; //BUF_STOP1按键次数
extern uint32_t HUST_BUTTOM_STOP2_STATUS_COUNT_ONE ; //BUF_STOP1按键次数
extern uint32_t HUST_BUTTOM_STOP1_STATUS_FLAG ;      //BUF_STOP1标志位
extern uint32_t HUST_BUTTOM_STOP2_STATUS_FLAG ;      //BUF_STOP2标志位

 /***************************************
* 函数功能：检测是否有按键按下
 ***************************************/
static StateStatus KeyStatus_Once(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t Down_state);

/******************************************************************************
  * 函数名  ：  KeyStatus_Continue
  * 函数功能：  检测按键是否被长按
 ******************************************************************************/
static Continue_StateStatus KeyStatus_Continue(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t Down_state);

/******************************************************************************
  * 函数名  ：  Buttom_Status_Continue_Down
  * 函数功能：  检测按键是否被长按后没有松开
 ******************************************************************************/
static Continue_State_Down_Status Buttom_Status_Continue_Down(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t Down_state);

/******************************************************************************
  * 函数名  ：  Buttom_Status_Continue_Release
  * 函数功能：  检测按键是否被长按后没有松开
 ******************************************************************************/
static Continue_State_Release_Status Buttom_Status_Continue_Release(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,uint8_t Down_state);

 /***************************************
* 函数功能：记录BUT_START按键按下次数
 ***************************************/
void HUST_BUTTOM_START_STATUS(void);

 /***************************************
* 函数功能：记录BUT_STOP按键按下次数
 ***************************************/
void HUST_BUTTOM_STOP_STATUS(void);

 /***************************************
* 函数功能：记录BUT_STOP1按键按下次数
 ***************************************/
void HUST_BUTTOM_STOP1_STATUS(void);

 /***************************************
* 函数功能：记录BUT_STOP2按键按下次数
 ***************************************/
void HUST_BUTTOM_STOP2_STATUS(void);

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
