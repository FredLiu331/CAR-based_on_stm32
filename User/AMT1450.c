#include "AMT1450.h"
#include "tm_stm32f4_i2c.h"
#include "delay.h"
#include "BTModule.h"


void init_AMT1450(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_8;				 //PB9,SDA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//推挽输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);

  TM_I2C_Init(AMT1450_I2Cx, TM_I2C_PinsPack_2, 400000);
	
  Delay_ms(10);
#if 1           //从机默认使用压缩数据及开启LED，如需修改模式，取消注释并配置功能使能寄存器
	config_AMT1450(AMT1450_Slave_Address0);  
	Delay_ms(5);
#endif
}
/**
  * @brief  AMT1450初始化:配置功能使能寄存器
  * @param  slave_addr：从机地址
  * @retval None
  */
void config_AMT1450(uint8_t slave_addr)
{
   //功能使能寄存器：第7位：LED；第6位：8点；第5位：24点；第4位：145点；第3位：压缩数据；第1位：校准
  uint8_t data=0x00;
	data=(1<<7)|(1<<3);       //默认打开LED及压缩数据,其他模式自行配置
	TM_I2C_Write(AMT1450_I2Cx,slave_addr,REG_Function_Enable,data);
}


/****************************读数据命令**********************************/
//////////////////////////////////////////////////////////////////////////
/**
  * @brief  获取压缩数据
  * @param  slave_addr 目标从机地址
  * @retval None
  */
void AMT1450_getCompressData(uint8_t slave_addr,uint8_t *begin_Color,uint8_t *jump_Count,uint8_t *jump_Location)
{
	uint8_t temp;
  //读取数据
  temp = TM_I2C_Read(AMT1450_I2Cx, slave_addr, REG_Jump_NUM);
  *begin_Color=temp&0x80;      //最开始的颜色  黑：1  白：0
  *jump_Count=temp&0x0F;       //跳变次数
	if(*jump_Count == 1)  //只有一个跳变
	{
		*jump_Location = TM_I2C_Read(AMT1450_I2Cx, slave_addr, REG_Jump_LOC);
	}
	else if(*jump_Count > 1)
	{
		if(*jump_Count > 8)   //最多读到8个跳变位置
			temp = 8;
		else
			temp = *jump_Count;
		TM_I2C_ReadMulti(AMT1450_I2Cx,slave_addr,REG_Jump_LOC,jump_Location,*jump_Count);  //每个字节表示一个跳变位置
	 
	}
	else
	{
		*jump_Location = 0;
	}
}

/**
  * @brief  获取8点数据
  * @param  salve_addr 目标从机地址, *p 1个字节缓冲区的变量指针
  * @retval None
  */
void AMT1450_getPoint8(uint8_t slave_addr,uint8_t *p)
{
	*p = TM_I2C_Read(AMT1450_I2Cx, slave_addr, REG_Eight_Data);
}
/**
  * @brief  获取24点数据
  * @param  salve_addr 目标从机地址, *p 3个字节缓冲区的变量指针
  * @retval None
  */
void AMT1450_getPoint24(uint8_t slave_addr,uint8_t *p)
{
	TM_I2C_ReadMulti(AMT1450_I2Cx,slave_addr,REG_TW_Four_Data,p,3);
}
/**
  * @brief  获取所有点数据
  * @param  salve_addr 目标从机地址, *p 18个字节缓冲区的变量指针
  * @retval None
  */
void AMT1450_getPointAll(uint8_t slave_addr,uint8_t *p)
{
	TM_I2C_ReadMulti(AMT1450_I2Cx,slave_addr,REG_All_Data,p,18);
}


/***************************修改/获取地址************************/
//////////////////////////////////////////////////////////////////
/**
  * @brief  修改从机地址
  * @param  new_addr 要修改的地址
  * @retval None
  */
void AMT1450_writeAddr(uint8_t new_addr)
{
	//向广播地址0x00写数据
	TM_I2C_Write(AMT1450_I2Cx,AMT1450_Slave_Common,REG_Slave_Address_W,new_addr);
}
/**
  * @brief  获取从机地址
  * @param  addr 要修改的地址
  * @retval 读到的地址
  */
uint8_t AMT1450_getAddr(void)
{
	//向广播地址读从机地址
  return TM_I2C_Read(AMT1450_I2Cx,AMT1450_Slave_Common,REG_Slave_Address_R);
}


//测试
void test_AMT1450(void)
{
 while(1)
	{
#if 1
		uint8_t begin,jump,count[2];
	  AMT1450_getCompressData(0x02,&begin,&jump,count);
//		//printf("%d\n",jump);
		 printf("%d %d\n",count[0],count[1]);
#endif
#if 0
		uint8_t temp;
		AMT1450_getPoint8(0x02,&temp);
		//printf("%d\n",temp);
		for(uint8_t i=0;i<8;i++){
		  printf("%d ",temp>>7);
			temp<<=1;		
		}
		printf("\n");
#endif
		#if 0
		uint8_t temp[3],temp0;
		AMT1450_getPoint24(0x02,temp);
		for(uint8_t j=0;j<3;j++){
			temp0=temp[j];
		for(uint8_t i=0;i<8;i++){
		  printf("%d ",temp0>>7);
			temp0<<=1;		
		}
		
		}	
		printf("\n");
#endif	
		Delay_ms(20);
	}
}



