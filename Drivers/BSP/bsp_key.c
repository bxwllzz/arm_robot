/******************************************************************************
  * 文件名  ：  bsp_key.c
  * 文件功能：  检测按键是否被按下以及记录按下次数
 ******************************************************************************/ 
#include "bsp_key.h"
//记录当前按键次数
uint32_t Key_statenumber[KEY_NUMBER] ;
uint8_t Down_State = 0 ;                      //按键按下时的电平，1为高电平，0为低电平
uint32_t HUST_BUTTOM_START_STATUS_COUNT = 0 ; //BUF_START按键次数
uint32_t HUST_BUTTOM_STOP_STATUS_COUNT  = 0 ; //BUF_STOP按键次数
uint32_t HUST_BUTTOM_STOP1_STATUS_COUNT = 0 ; //BUF_STOP1按键次数
uint32_t HUST_BUTTOM_STOP2_STATUS_COUNT = 0 ; //BUF_STOP1按键次数

/******************************************************************************
  * 函数名  ：  Key_Scan
	* 输入变量：  引脚类别，引脚位置，状态指示
  * 输出值  ：  按下输出KEY_ON，没按下输出KEY_OFF
  * 函数功能：  检测按键是否被按下后复位
 ******************************************************************************/ 
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin ,uint8_t Down_state)
{
	uint32_t tickstart = 0;
	if( HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == Down_state )
	{
		//获取当高/低电平开始时的时间
		tickstart = HAL_GetTick();
		////获取当高/低电平持续的时间
		while( HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == Down_state );
		//持续时间阈值处理
		if( HAL_GetTick() - tickstart > 100)
			return KEY_ON;
		else 
			return KEY_OFF;
	}
	else
		return KEY_OFF;
}

/******************************************************************************
  * 函数名  ：  Key_Scan_Clock
	* 输入变量：  引脚类别，引脚位置，状态指示
  * 输出值  ：  按下输出KEY_ON，没按下输出KEY_OFF
  * 函数功能：  检测按键是否被按下后不复位
 ******************************************************************************/ 
uint8_t Key_Scan_Clock(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin ,uint8_t Down_state)
{
	uint32_t tickstart = 0;
	uint8_t  flag = 0; //标记位
	if( HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == Down_state )
	{
		//获取当高/低电平开始时的时间
		tickstart = HAL_GetTick();
		////获取当高/低电平持续的时间
		while( HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == Down_state )
		{
			if( HAL_GetTick() - tickstart > 1000) //持续时间阈值处理
			{
				flag = 1;
				break;
			}
		} 
		if( 1 == flag)
		{
			return KEY_ON;
		}
		else
			return KEY_OFF;
	}
	else
		return KEY_OFF;
}

/******************************************************************************
  * 函数名  ：  Key_Number
	* 输入变量：  引脚类别，引脚位置，状态指示，引脚数目
  * 输出值  ：  无
  * 函数功能：  检测按键按下次数
 ******************************************************************************/  
void HUST_BUTTOM_STATUS_COUNT( GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin ,uint8_t Down_state, uint8_t GPIO_Number)
{
	if( Key_Scan( GPIOx, GPIO_Pin, Down_state) == KEY_ON)
	{
		Key_statenumber[GPIO_Number]++;
	}
}

/******************************************************************************
  * 函数名  ：  BUF_START
	* 输入变量：  无
  * 输出值  ：  无
  * 函数功能：  检测BUF_START按键按下次数
 ******************************************************************************/ 
void BUF_START(void)
{
	if( Key_Scan( GPIOC, GPIO_PIN_10, Down_State ) == KEY_ON)
	{
		HUST_BUTTOM_START_STATUS_COUNT++;
	}
}

/******************************************************************************
  * 函数名  ：  BUF_STOP
	* 输入变量：  无
  * 输出值  ：  无
  * 函数功能：  检测BUF_STOP按键按下次数
 ******************************************************************************/ 
void BUF_STOP(void)
{
	if( Key_Scan( GPIOC, GPIO_PIN_11, Down_State ) == KEY_ON)
	{
		HUST_BUTTOM_STOP_STATUS_COUNT++;
	}
}

/******************************************************************************
  * 函数名  ：  BUF_STOP1/2
	* 输入变量：  无
  * 输出值  ：  无
  * 函数功能：  检测BUF_STOP1/2P按键按下次数
 ******************************************************************************/ 
void BUF_STOP1(void)
{
	if( Key_Scan_Clock( GPIOA, GPIO_PIN_1, Down_State ) == KEY_ON)
	{
		HUST_BUTTOM_STOP1_STATUS_COUNT++;
	}
}

void BUF_STOP2(void)
{
	if( Key_Scan_Clock( GPIOA, GPIO_PIN_0, Down_State ) == KEY_ON)
	{
		HUST_BUTTOM_STOP2_STATUS_COUNT++;
	}
}




