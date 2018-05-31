/******************************************************************************
  * 文件名  ：  keyboard.c
  * 文件功能：  检测按键是否被按下以及记录按下次数（状态机）
 ******************************************************************************/ 
#include "keyboard.h"

//KEY_DOWN_TIME和KEY_HOLD_TIME根据调试情况来确定
#define KEY_MAX  4	                                    //有4个按键
#define KEY_MSG_FIFO_MAX   200           	              //最大存储的键盘信息
uint32_t KEY_HOLD_TIME   =   120;//20000;	             	//用于判断按键按下的时间，当大于这个时间后就认为是长按
uint32_t KEY_DOWN_TIME   =	 40;//2500;				          //用于判断按键按下的时间，当大于这个时间后就认为是短按
KEY_MSG KEY_MSG_FIFO[KEY_MSG_FIFO_MAX];   		          //用来储存按键的消息
uint32_t KEY_Press_Time[KEY_MAX];
uint8_t key_msg_rear = 0,key_msg_front = 0;             //key_msg_rear用来指示当前储存的按键消息
														                            //key_msg_front用来指示当前所取的按键消息
volatile KEY_MSG_STATUS key_msg_status = KEY_MSG_EMPTY; //key_msg_status用来指示当前消息队列的状态


//储存按键消息
uint8_t Send_KeyMsg(KEY_MSG key_msg)
{
	if(key_msg_status == KEY_MSG_FULL)		//消息队列里的消息已经存满
	{
		return 0;
	}
	
	//储存按键消息
	KEY_MSG_FIFO[key_msg_rear].status = key_msg.status;
	KEY_MSG_FIFO[key_msg_rear].key    = key_msg.key;
	key_msg_rear++;
	
	if(key_msg_rear >= KEY_MSG_FIFO_MAX) //大于最大消息数，清零从头再储存
	{
		key_msg_rear = 0;			
	}
	if(key_msg_rear == key_msg_front)	   //一般情况是key_msg_rear>key_msg_front的，如果一直输入消息
	{									                   //使key_msg_rear加到和key_msg_front相等时，这个时候是不能再
		key_msg_status = KEY_MSG_FULL;	   //储存消息的，因为消息都没被取完
	}
	else
	{
		key_msg_status = KEY_MSG_NORMAL;
	}
	return 1;
}

//获取消息队列里的按键消息
uint8_t Get_KeyMsg(KEY_MSG *key_msg)
{
	if(key_msg_status == KEY_MSG_EMPTY)		//消息队列里的消息已经取完
	{
		return 0;
	}
	
	//取消息
	key_msg->status = KEY_MSG_FIFO[key_msg_front].status;
	key_msg->key    = KEY_MSG_FIFO[key_msg_front].key;
	key_msg_front++;
	
	if(key_msg_front >= KEY_MSG_FIFO_MAX)  //大于最大消息数，清零从头再储存
	{
		key_msg_front = 0;			
	}
	
	if(key_msg_front == key_msg_rear)	  //当两者相等时，说明消息队列里已经没有消息了
	{
		key_msg_status = KEY_MSG_EMPTY;
	}
	return 1;
}

//检查哪个按键被按下去，记录时间
void Check_Key(void)
{
	KEY_MSG key_msg ;
	uint8_t KEY_IN = 0;
	uint8_t col;
	if( HAL_GPIO_ReadPin(KEYC_PORT,KEYC_10_PIN) == 0 || HAL_GPIO_ReadPin(KEYC_PORT,KEYC_11_PIN) == 0 \
		  || HAL_GPIO_ReadPin(KEYA_PORT,KEYA_0_PIN) == 0  || HAL_GPIO_ReadPin(KEYA_PORT,KEYA_1_PIN) == 0 )
	{
		for(col = 0;col < 4;col++)
		{
				if(col == 3)
					KEY_IN = HAL_GPIO_ReadPin(KEYA_PORT,KEYA_0_PIN);
				else if(col == 2)
					KEY_IN = HAL_GPIO_ReadPin(KEYA_PORT,KEYA_1_PIN);
				else if(col == 1)
					KEY_IN = HAL_GPIO_ReadPin(KEYC_PORT,KEYC_11_PIN);
				else if(col == 0)
					KEY_IN = HAL_GPIO_ReadPin(KEYC_PORT,KEYC_10_PIN);
				if( KEY_IN == 0x00)
				{
					KEY_Press_Time[col]++; 
					if(KEY_Press_Time[col]<=KEY_DOWN_TIME) //一直循环直到达到预设的时间
					{
						continue;
					}
					else if(KEY_Press_Time[+col]==KEY_DOWN_TIME+1)
					{
						key_msg.key=(KEY_Type)(col);
						key_msg.status=KEY_DOWN;
						Send_KeyMsg(key_msg);
					}
					else if(KEY_Press_Time[col]<=KEY_HOLD_TIME)
					{
						continue;
					}
					else if(KEY_Press_Time[col]==KEY_HOLD_TIME+1)
					{
						key_msg.key=(KEY_Type)(col);   
						key_msg.status=KEY_HOLD;
						Send_KeyMsg(key_msg);
					}
					else
					{
						KEY_Press_Time[col] = KEY_DOWN_TIME+1;
					}
				}
				else
				{
					key_msg.key=(KEY_Type)(col); 
					key_msg.status=KEY_IDLE;
					KEY_Press_Time[col]=0;
				}
		}
		
	}
	else
	{
		for(int i=0;i<4;i++)
		{
			key_msg.key=(KEY_Type)(i); 
			key_msg.status=KEY_IDLE;
			KEY_Press_Time[i]=0;
			//Send_Key_msg(key_msg); 
		}
	}	
}


