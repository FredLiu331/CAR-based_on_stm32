#ifndef __AMT1450_H
#define __AMT1450_H
#include "stm32f4xx.h"


#define AMT1450_I2Cx                   I2C1        //ʹ��I2C1

#define AMT1450_Slave_Common          0x00        //I2C�㲥��ַ
#define AMT1450_Slave_Address0        0x02        //��һ��
#define AMT1450_Slave_Address1        0x06        //��һ��
#define AMT1450_Slave_Address2        0x1a        //��һ��
#define AMT1450_Slave_Address3        0x1e        //��һ��


/*******************WO����д********************/
#define REG_Slave_Address_W           0x01       //�洢�ӻ������ַ  //�ӻ��ĵ�ַ����flash���޸�
#define REG_Function_Enable           0x02       //����ʹ�ܼĴ�������7λ��LED����6λ��8�㣻��5λ��24�㣻��4λ��145�㣻��3λ��ѹ�����ݣ���1λ��У׼

/*********************RO���ɶ�**********************/
#define REG_Slave_Address_R           0x03       //�洢�ɶ��Ĵӻ������ַ,��Writeλ��һ��
#define REG_Eight_Data                0x04       //
#define REG_TW_Four_Data              0x05       //
#define REG_All_Data                  0x08       //0x08~0x19
#define REG_Jump_NUM                  0x1a       //��ƽ�������(����λΪ����λ) �����λΪ�������ҵ�һ����ɫ���ڣ�1���ף�0��
#define REG_Jump_LOC                  0x1b       //0x1b~0x22��ƽ����λ��   31


#define myabs(amt) ((amt)<(0)?(-amt):(amt))


//��ʼ�����ò���
void init_AMT1450(void);  //I2C��ʼ��
void config_AMT1450(uint8_t slave_addr);  //AMT1450��ʼ��

//������
void AMT1450_getCompressData(uint8_t slave_addr,uint8_t *begin_Color,uint8_t *jump_Count,uint8_t *jump_Location);
void AMT1450_getPoint8(uint8_t slave_addr,uint8_t *p);          //��ȡ8�����ݣ�pΪһ���ֽڵı���
void AMT1450_getPoint24(uint8_t slave_addr,uint8_t *p);          //��ȡ24�����ݣ�pΪ3���ֽڵĻ�����
void AMT1450_getPointAll(uint8_t slave_addr,uint8_t *p);          //��ȡ���е����ݣ�pΪ18���ֽڵĻ�����

//����
void AMT1450_writeAddr(uint8_t new_addr);             //�޸Ĵӻ���ַ
uint8_t AMT1450_getAddr(void);  //��ȡ�ӻ���ַ


//����
void test_AMT1450(void);

#endif
