
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __keyboard_H
#define __keyboard_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "gpio.h"

#define KEYC_PORT  GPIOC
#define KEYA_PORT  GPIOA	

#define KEYA_0_PIN  0  
#define KEYA_1_PIN  1   
#define KEYC_10_PIN 10  
#define KEYC_11_PIN 11  	 

//按键的种类，以下名称到时候要修改
typedef enum 
{    
	BUF_START_KEY,
	BUF_STOP_KEY,
	BUF_STOP1_KEY,
	BUF_STOP2_KEY,
}KEY_Type;	 
	 
//按键的状态
typedef enum
{
	KEY_IDLE,         		//空闲态
  KEY_DOWN,       			//按下按键
	KEY_HOLD,				      //保持
}KEY_Status;

//按键类
typedef struct
{
	KEY_Type 	key;			  //哪个按键
	KEY_Status  status;   //按键的状态
}KEY_MSG;

//消息的状态
typedef enum
{
    KEY_MSG_NORMAL,    //一般消息
    KEY_MSG_FULL,      //消息已满
    KEY_MSG_EMPTY,     //消息空
}KEY_MSG_STATUS;

uint8_t Send_KeyMsg(KEY_MSG key_msg);
uint8_t Get_KeyMsg(KEY_MSG *key_msg);
void Check_Key(void);

extern uint32_t KEY_HOLD_TIME;						//用于判断按键按下的时间，当大于这个时间后就认为是长按
extern uint32_t KEY_DOWN_TIME;							

	 
	 
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
