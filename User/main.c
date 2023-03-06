//#include "stm32f4xx.h"
#include "misc.h"
#include "delay.h"
#include "KeyLed.h"
#include "MotorDriver.h"
#include "MotorController.h"
#include "AMT1450.h"
#include "amt1450_uart.h"
#include "BTModule.h"
#define HIGH_STD_SPEED 500

void NVIC_Configuration(void); //中断配置
void GPIO_Config(void); //通用输入输出端口配置

//全局变量
uint16_t n10msCount;
uint8_t b100msFlag;
uint16_t n100msCount;

int16_t nSpeedCnt;
int32_t nSpeed;
int key=1;
//***************************************************
//数据结构&参数设定 
//***************************************************
int trackSpeed;//循迹速度

typedef struct Track_PID
{
	int target;//设定目标
	long error_acc;//误差累计值  
	double track_KP;//比例系数
	double track_KI;//积分系数
	double track_kD;//微分系数

	int lastError;
	int prevError;
}Track_PID;

static Track_PID sPID;
static Track_PID *PIDpoint=&sPID;//这是一个指针！！
extern uint8_t begin,jump,count[6];//1/黑；0/白
extern uint8_t position;

//历史探测数据，1表示上一次没有检测到路口，0表示上一次检查到路口
int8_t trackHistory;
//十字路口计数
int8_t nCrossing;

//****************************************************
//路线与动作 *
//****************************************************

const double chaspeed=620;//左右轮之间差速度计算值(等待调整) 测出两者之间的比值
const double turningspeed=550;
void TurnLeft(void){
			MotorController_SetSpeed(2,0);
			MotorController_SetSpeed(1,0); 
			Delay_ms(30);
			MotorController_SetSpeed(2,-turningspeed+chaspeed);
			MotorController_SetSpeed(1,turningspeed+chaspeed);  //假设2是左边轮子 假设1是右边轮子
		  Delay_ms(280);    //时间需要调整

}
void TurnRight(void){
			MotorController_SetSpeed(2,0);
			MotorController_SetSpeed(1,0); 
			Delay_ms(30);
			MotorController_SetSpeed(2,-turningspeed-chaspeed);// 假设1是右边轮子 假设2是左边轮子
			MotorController_SetSpeed(1,turningspeed-chaspeed);  
		  Delay_ms(280);    //时间需要调整

}	
void Gostratge(void){
			
			MotorController_SetSpeed(2,-trackSpeed);// 假设1是右边轮子 假设2是左边轮子
			MotorController_SetSpeed(1,trackSpeed);  
		  
			Delay_ms(30);    //时间需要调整

}	



const int Map_load[]={0,0,1,0,1,3,1,1,3,1,0,0,1,1,1,1,1,1,1,0,0,1,3,1,0,0,2,0,0,0,0,0,2,0,0,0,0,0,0,2,4};
//const int Map_load[]={0,0,0,2,2,1,1,1,1,2,2,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,2,2,0,0,0,0,0,2,0,1,3,1,1,0,2,0,0,2,0,0,0,0,0,1,1,1,1,0,0,0,3,0,0,2,2,1,1,1,1,2,2,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,2,2,0,0,0,0,0,2,0,1,3,1,1,0,2,0,0,2,0,0,0,0,0,1,1,1,1,0,0,0,4};
//const int Map_loadfan[]={0,0,0,2,2,1,1,1,1,2,2,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,2,2,0,0,0,0,0,2,0,1,3,1,1,0,2,0,0,2,0,0,0,0,0,1,1,1,1,0,0,0,4};

//const int Map_load[]={0,0,0,1,3,4}; 

/*
void Map_load(int nCrossing)
	{//设置 0 为直走 1为左转 2为右转 3为掉头 4停止
		int a[]={0,0,0,2,2,1,1,1,2,2,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,2,2,0,0,0,0,0,2,0,1,3,1,1,0,2,0,0,2,0,0,0,0,0,1,1,1,1,0,0,0,4};
	    if (a[nCrossing]==0){}
				else if(a[nCrossing]==1){
							
				}else if(a[nCrossing]==2){}
						else if(a[nCrossing]==3){}
							else if (a[nCrossing]==4){}
			

}*/
	

void Map_read(int nCross){
	
	 if (Map_load[nCross]==0){
		MotorController_SetSpeed(1,-trackSpeed);
		MotorController_SetSpeed(2,trackSpeed);
	 }
				else if(Map_load[nCross]==1){
					TurnLeft();			
				}else if(Map_load[nCross]==2){
					TurnRight();	
				}else if(Map_load[nCross]==3)//需要掉头行驶的地方 设置先转向 再先前进，在后退，在转向
							{
									MotorController_SetSpeed(2,0);
									MotorController_SetSpeed(1,0);
									Delay_ms(50);
								
									MotorController_SetSpeed(2,620);// 假设1是右边轮子 假设2是左边轮子
									MotorController_SetSpeed(1,+620);
									Delay_ms(480);                  // 旋转时间需要调整 彻底旋转一圈
											
						}else if (Map_load[nCross]==4){ 
									key=0;
							}
}


//*************PID参数初始化**************************
void PIDinit(void)
{
	PIDpoint->error_acc=0;
	PIDpoint->lastError=0;
	PIDpoint->prevError=0;

	PIDpoint->target=72;//希望位置保持在72（1~144,72为中间位置）
	PIDpoint->track_KP=2.2;
	PIDpoint->track_KI=0;//不使用积分
	PIDpoint->track_kD=1.0;
}

//****************************************************
//循迹初始化 *
//****************************************************
void Trackinit(void)
{
	trackSpeed=HIGH_STD_SPEED;//巡航速度，定义到了MotorController.h
	trackHistory=0;
	nCrossing=0;
	PIDinit();//初始化pid参数
	//Map_load(nCrossing);
	//路线程序
	MotorController_SetSpeed(1,-trackSpeed);
	MotorController_SetSpeed(2,trackSpeed);
}

//****************************************************
//位置式P(I)D计算子函数 *传入得到的位置信息
//****************************************************
int TrackControl_PID(int input)
{
	int iError,output=0;
	iError=PIDpoint->target-input;
	PIDpoint->error_acc+=iError;
	output=PIDpoint->track_KP*iError+PIDpoint->track_KI*PIDpoint->error_acc*0.5f+PIDpoint->track_kD*iError-PIDpoint->lastError;
	PIDpoint->lastError=iError;
	return output;
}

//*****************************************************
//十字路口判定 *
//*****************************************************
int CrossJudging(void)
{
	int8_t out=0;
	if(trackHistory==1&&begin==0&&jump==0)//表示上一次检测不是路口，最左端为白色表示检测到横向白线
	{
		trackHistory=0;
		nCrossing+=1;
	printf("%d",nCrossing);
		out=1;
	}
	if(trackHistory==0&&begin==1) trackHistory=1;
	return out;
}

//****************************************************
//循迹子函数*
//****************************************************
void TrackControl(void)
{
	while(1)
	{
		get_AMT1450Data_UART(&begin,&jump,count);
		if(jump==2) position=0.5f*(count[0]+count[1]);
		MotorController_SetSpeed(2,-trackSpeed+TrackControl_PID(position));
		MotorController_SetSpeed(1,trackSpeed+TrackControl_PID(position));
		if(CrossJudging()==1) break;//跳出循环，结束循迹
		Delay_ms(10);
		
	}
}



int main(void)
{
	n100msCount = 0;

	GPIO_Config(); //端口初始化
	NVIC_Configuration();  //中断初始化
	Led_Init(2); //只使用LED2，LED3的引脚可以做其他用途
	Key_Init(2); //只使用Key1，Key2的引脚可以做其他用途
	key=1;

	init_AMT1450_UART();			// 初始化amt1450串口通信，串口使用UART5
	USART2_Init();
	Delay_ms(100);
	while(Key_Released(1)==0){}  //如果Key1没有按下，则一直等待
	Delay_ms(10);
	AMT1450_UART_Cmd(ENABLE);
//	amt1450_Test_UART();        //测试AMT1450，在while中

	
	MotorDriver_Init(4);
	MotorDriver_Start(1,PWM_DUTY_LIMIT/2);
	MotorDriver_Start(2,PWM_DUTY_LIMIT/2);
	MotorDriver_Start(3,PWM_DUTY_LIMIT/2);
	MotorDriver_Start(4,PWM_DUTY_LIMIT/2);
	Encoder_Init(4);

	MotorController_Init(330,72,4);  //初始化调速器，参数1：轮子转一圈输出的脉冲个数；参数2：轮子直径，单位mm；参数3：几个电机需要调速
	MotorController_Enable(ENABLE);
	MotorController_SetAcceleration(8000);  //设置加速度值，单位：mm/秒*秒

	printf("%d",nCrossing);
	Trackinit();//启动循迹初始化，路线载入，电机开始工作
	Delay_ms(100);

	//nSpeed = 0;

	
	
	while(1)
	{
		TrackControl();
		Map_read(nCrossing);
		if(key==0)
		{
			MotorController_SetSpeed(2,0);
			MotorController_SetSpeed(1,0);
			Delay_ms(50);
			return 0;
		}
		
		
	/*	if(Key_Released(1)==1) 
		{
			nSpeed += 100;
			MotorController_SetSpeed(1,-nSpeed);
			MotorController_SetSpeed(2,nSpeed);
		}
		if(Key_Released(2)==1) 
		{
			nSpeed -= 100;
			MotorController_SetSpeed(1,-nSpeed);
			MotorController_SetSpeed(2,nSpeed);   
		}
	
		LED2_ON();
		Delay_ms(1000);
		LED2_OFF();
		Delay_ms(1000);
	*/
	}
}




/*中断配置函数*/
void NVIC_Configuration(void)
{
	/* Configure one bit for preemption priority */
	/* 优先级组 说明了抢占优先级所用的位数，和子优先级所用的位数。这里是2，2*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//设置嘀嗒时钟中断
	if (SysTick_Config(SystemCoreClock / 100000))
	{
		/* Capture error */

		while (1)
			;
	}
}

/*端口配置函数*/
void GPIO_Config(void)
{
	//使能GPIOA/GPIOC的总线，否则端口不能工作，如果不用这个端口，可以不用使能。
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	  // PB2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //推挽输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


