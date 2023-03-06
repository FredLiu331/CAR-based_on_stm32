#ifndef __AMT1450_H
#define __AMT1450_H
#include "stm32f4xx.h"


#define AMT1450_I2Cx                   I2C1        //使用I2C1

#define AMT1450_Slave_Common          0x00        //I2C广播地址
#define AMT1450_Slave_Address0        0x02        //第一条
#define AMT1450_Slave_Address1        0x06        //第一条
#define AMT1450_Slave_Address2        0x1a        //第一条
#define AMT1450_Slave_Address3        0x1e        //第一条


/*******************WO仅可写********************/
#define REG_Slave_Address_W           0x01       //存储从机自身地址  //从机的地址可在flash中修改
#define REG_Function_Enable           0x02       //功能使能寄存器：第7位：LED；第6位：8点；第5位：24点；第4位：145点；第3位：压缩数据；第1位：校准

/*********************RO仅可读**********************/
#define REG_Slave_Address_R           0x03       //存储可读的从机自身地址,与Write位的一致
#define REG_Eight_Data                0x04       //
#define REG_TW_Four_Data              0x05       //
#define REG_All_Data                  0x08       //0x08~0x19
#define REG_Jump_NUM                  0x1a       //电平跳变次数(低四位为数据位) ，最高位为从左往右第一个颜色（黑：1；白：0）
#define REG_Jump_LOC                  0x1b       //0x1b~0x22电平跳变位置   31


#define myabs(amt) ((amt)<(0)?(-amt):(amt))


//初始化配置部分
void init_AMT1450(void);  //I2C初始化
void config_AMT1450(uint8_t slave_addr);  //AMT1450初始化

//读数据
void AMT1450_getCompressData(uint8_t slave_addr,uint8_t *begin_Color,uint8_t *jump_Count,uint8_t *jump_Location);
void AMT1450_getPoint8(uint8_t slave_addr,uint8_t *p);          //获取8点数据，p为一个字节的变量
void AMT1450_getPoint24(uint8_t slave_addr,uint8_t *p);          //获取24点数据，p为3个字节的缓冲区
void AMT1450_getPointAll(uint8_t slave_addr,uint8_t *p);          //获取所有点数据，p为18个字节的缓冲区

//命令
void AMT1450_writeAddr(uint8_t new_addr);             //修改从机地址
uint8_t AMT1450_getAddr(void);  //获取从机地址


//测试
void test_AMT1450(void);

#endif
