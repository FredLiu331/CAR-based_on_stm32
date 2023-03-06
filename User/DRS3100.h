#ifndef __DRS3100_H
#define __DRS3100_H
#include "stm32f4xx.h"


/*BT04蓝牙串口透传模块*/

//使用USART2，对应PA2/TX，PA3/RX，

typedef enum {DRS3100_USART = 0, DRS3100_I2C = 1} DRS3100_COM_Type;
#define DRS3100_I2Cx I2C1
#define DRS3100_I2C_Address 0x54
#define DRS3100_FRAME_BYTE_LENGTH 19 //串口通讯一帧数据的字节数（含帧头和帧尾），譬如20个字节为一个完整的数据帧，第1个字节帧头，第2个字节代表命令类型，第3~6字节是命令参数，第7个字节为帧尾
#define DRS3100_FRAME_START 0x55 //帧头


//这里把数据帧接收标志和接收缓冲区定义成可供BTModule.c以外的源代码直接使用，只要include了BTModule.h文件。
extern __IO uint8_t DRS3100_Buff[DRS3100_FRAME_BYTE_LENGTH]; //接收缓冲区
extern __IO uint8_t DRS3100_FrameFlag; //接收完整数据帧标志，1完整，0不完整

void DRS3100_Init(DRS3100_COM_Type ComType);

void DRS3100_GetPoint8(uint8_t *p);
void DRS3100_GetPoint24(uint8_t *p);
void DRS3100_GetPoint139(uint8_t *p);
void DRS3100_GetTurn(uint8_t *p);
void DRS3100_GetFlagLimit(uint8_t *p);

#endif
