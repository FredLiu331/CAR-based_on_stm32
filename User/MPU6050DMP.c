#include "MPU6050DMP.h"
#include "stdio.h"
#include "math.h"
#include "tm_stm32f4_i2c.h"

uint32_t nI2CTimeout;
/*在使用本库前，先在预编译符号定义中定义MPU6050和EMPL_TARGET_STM32*/
/*在delay.c中也有部分本库需要的代码，注意#if defined MPU6050部分*/

/* Buffer of data to be received by I2C1 */
uint8_t IIC_Buffer_Rx[25];
/* Buffer of data to be transmitted by I2C1 */
uint8_t IIC_Buffer_Tx[25] = {0x5, 0x6,0x8,0xA};

//变量定义，注释掉的变量定义全部移入函数内了。

//struct int_param_s int_param;

//unsigned short gyro_rate, gyro_fsr;
//unsigned char accel_fsr;
static signed char gyro_orientation[9] = {1,  0,  0,
                                           0, 1,  0,
                                           0,  0,  1};

//short gyro[3], accel[3], sensors;
//unsigned char more;
//long quat[4];
//unsigned long timestamp;
	
////struct dmpEuler_s sEuler;
//struct dmpGravity_s sGravity;
//struct dmpQuaternion_s sQuaternion;
//struct dmpYawPitchRoll_s sYawPitchRoll;
//变量定义结束

//**************************************
//初始化MPU6050
//**************************************
int MPU6050_Init(void)
{
//	Status rtn;
//	NVIC_InitTypeDef NVIC_InitStructure;  //NVIC中断向量结构体

//	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;					//配置I2C1EV中断
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure); 
//	
//	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;					//配置I2C1ER中断 
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure); 
	
	TM_I2C_Init(I2C1, TM_I2C_PinsPack_2, 300000); //I2C_LowLevel_Init(MPU6050_I2C);
	//Single_WriteI2C(PWR_MGMT_1, 0x00);	//解除休眠状态
	IIC_Buffer_Tx[0]=PWR_MGMT_1;
	IIC_Buffer_Tx[1]=0x01;
  //rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	//if(rtn !=Success) return 0;
	TM_I2C_Write(MPU6050_I2C, MPU6050_Addr, PWR_MGMT_1, 0x01);
	
	//Single_WriteI2C(SMPLRT_DIV, 0x07);
	IIC_Buffer_Tx[0]=SMPLRT_DIV;
	IIC_Buffer_Tx[1]=0x31;
  //rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	//if(rtn !=Success) return 0;
	TM_I2C_Write(MPU6050_I2C, MPU6050_Addr, SMPLRT_DIV, 0x31);
	
	//Single_WriteI2C(MPU6050_CONFIG, 0x06);
	IIC_Buffer_Tx[0]=MPU6050_CONFIG;
	IIC_Buffer_Tx[1]=0x06;
  //rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	//if(rtn !=Success) return 0;
	TM_I2C_Write(MPU6050_I2C, MPU6050_Addr, MPU6050_CONFIG, 0x06);
	
	//Single_WriteI2C(GYRO_CONFIG, 0x18);
	IIC_Buffer_Tx[0]=GYRO_CONFIG;
	IIC_Buffer_Tx[1]=0x18;
  //rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	//if(rtn !=Success) return 0;
	TM_I2C_Write(MPU6050_I2C, MPU6050_Addr, GYRO_CONFIG, 0x18);
	
	//Single_WriteI2C(ACCEL_CONFIG, 0x01);
	IIC_Buffer_Tx[0]=ACCEL_CONFIG;
	IIC_Buffer_Tx[1]=0x01;
  //rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	//if(rtn !=Success) return 0;
	TM_I2C_Write(MPU6050_I2C, MPU6050_Addr, ACCEL_CONFIG, 0x01);
	
	//配置Aux I2C 为 bypass mode , 0x37的bit1=1 而且 0x6A的bit
	IIC_Buffer_Tx[0] = 0x37;
	IIC_Buffer_Tx[1] = 0x02;  //0x37 / INT_LEVEL INT_OPEN LATCH _INT_EN INT_RD _CLEAR FSYNC_ INT_LEVEL FSYNC_ INT_EN I2C _BYPASS _EN CLKOUT _EN
	//rtn = I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	//if(rtn !=Success) return 0;
	TM_I2C_Write(MPU6050_I2C, MPU6050_Addr, 0x37, 0x02);
	
	IIC_Buffer_Tx[0] = 0x6A;  //0x6A  / - FIFO_EN[6] I2C_MST_EN[5] I2C_IF_DIS[4] - FIFO_RESET[2] I2C_MST_RESET[1] SIG_COND_RESET[0]
	IIC_Buffer_Tx[1] = 0x00; 
	//rtn = I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,2,Polling, MPU6050_Addr);
	//if(rtn !=Success) return 0;
	TM_I2C_Write(MPU6050_I2C, MPU6050_Addr, 0x6A, 0x00);
	
	return 1;
}

//**************************************
//读指定地址数据
//**************************************
uint16_t MPU6050_GetData(uint8_t REG_Address)
{
//	Status rtn;
	char H,L;
	  H=TM_I2C_Read(MPU6050_I2C, MPU6050_Addr, REG_Address);
//		IIC_Buffer_Tx[0]=REG_Address;
//	  I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,1,Polling, MPU6050_Addr);
//		rtn=I2C_Master_BufferRead(MPU6050_I2C,IIC_Buffer_Rx,1,Polling, MPU6050_Addr);	
//		if(rtn == Success) H=IIC_Buffer_Rx[0];
//		else H=0;
	  L=TM_I2C_Read(MPU6050_I2C, MPU6050_Addr, REG_Address+1);
//		IIC_Buffer_Tx[0]=REG_Address+1;
//	  I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,1,Polling, MPU6050_Addr);
//		rtn=I2C_Master_BufferRead(MPU6050_I2C,IIC_Buffer_Rx,1,Polling, MPU6050_Addr);
//		if(rtn == Success) L=IIC_Buffer_Rx[0];
//		else L=0;
	return (H<<8)+L;   //合成数据
}

//**************************************
//连续读ax,ay,az,temperature,gx,gy,gz数据
//**************************************
int MPU6050_ReadRawData(struct MPU6050_RawData_s *s_IMUVar)
{
	
	/* //下面是直接读寄存器的代码
	Status rtn;
	char H,L;

	IIC_Buffer_Tx[0]=ACCEL_XOUT_H;
  I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,1,Polling, MPU6050_Addr);
	rtn=I2C_Master_BufferRead(MPU6050_I2C,IIC_Buffer_Rx,14,DMA, MPU6050_Addr);	
	
	if(rtn == Success) 
	{
		H=IIC_Buffer_Rx[0];
		L=IIC_Buffer_Rx[1];
		s_IMUVar->ax = (H<<8)+L;
		
		H=IIC_Buffer_Rx[2];
		L=IIC_Buffer_Rx[3];
		s_IMUVar->ay = (H<<8)+L;
		
		H=IIC_Buffer_Rx[4];
		L=IIC_Buffer_Rx[5];
		s_IMUVar->az = (H<<8)+L;
		
		H=IIC_Buffer_Rx[6];
		L=IIC_Buffer_Rx[7];
		s_IMUVar->temperature = (H<<8)+L;
		
		H=IIC_Buffer_Rx[8];
		L=IIC_Buffer_Rx[9];
		s_IMUVar->gx = (H<<8)+L;
		
		H=IIC_Buffer_Rx[10];
		L=IIC_Buffer_Rx[11];
		s_IMUVar->gy = (H<<8)+L;
		
		H=IIC_Buffer_Rx[12];
		L=IIC_Buffer_Rx[13];
		s_IMUVar->gz = (H<<8)+L;

	}
	else
	{
		return Error;
	}
	return Success;
	*/

	  //下面是读FIFO的代码，陀螺仪数据有纠偏
	short gyro[3], accel[3], sensors;
	unsigned char more;
  long quat[4];
	unsigned long timestamp;
	int rtn;
	if(mpu_set_dmp_state(1))
	{
		return 0;
	}
	do
	{
		rtn=dmp_read_fifo( gyro, accel, quat, &timestamp, &sensors, &more );
	}
	while(more);
	
	if(!rtn)
	{
		if((sensors & INV_XYZ_ACCEL) || (sensors & INV_XYZ_GYRO))
		{
			s_IMUVar->ax = accel[0];
			s_IMUVar->ay = accel[1];
			s_IMUVar->az = accel[2];
			
			s_IMUVar->gx = gyro[0];
			s_IMUVar->gy = gyro[1];
			s_IMUVar->gz = gyro[2];
		}
		else
		{
			return 0;	
		}
	}
	else
	{
		return 0;
	}
	return 1;
	
}


int stm32_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data)
{
    unsigned char i;
//		Status rtn;
    if (!length)
        return 0;
		
		if(length > 24) return -1;
		
		IIC_Buffer_Tx[0] = reg_addr;
		for(i=0;i<length;i++)
		{
			IIC_Buffer_Tx[i+1]=data[i];
		}
//		rtn=I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,length+1,Polling, slave_addr);
//    if(rtn==Success)return 0;
//		else return -1;
		TM_I2C_WriteMulti(MPU6050_I2C, slave_addr, reg_addr, &IIC_Buffer_Tx[1], length);
		
		return 0;
}


int stm32_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data)
{
    if (!length)
        return 0;
		TM_I2C_ReadMulti(MPU6050_I2C, slave_addr, reg_addr, data, length);
		return 0;
}

unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

uint8_t MPU6050_SelfTest(void)
{
    int rtn;
    long gyro[3], accel[3];
		float sens;
		unsigned short accel_sens;

    rtn = mpu_run_self_test(gyro, accel);
    if (rtn == 0x3) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */

        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);

    }
		else
		{
			return 0;
		}
		return 1;
}

//
void dmpGetGravity(struct dmpGravity_s *v, struct dmpQuaternion_s *q) {
    v->x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v->y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v->z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
}

void dmpGetEuler(struct dmpEuler_s *data, struct dmpQuaternion_s *q) {
    data->psi = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data->theta = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data->phi = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
}

void dmpGetYawPitchRoll(struct dmpYawPitchRoll_s *data, struct dmpQuaternion_s *q, struct dmpGravity_s *gravity) {
    // yaw: (about Z axis) 偏航角
    data->yaw = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis) 俯仰角
    data->pitch = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis) 横滚角
    data->roll = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
}

int MPU6050_GetYawPitchRoll(struct dmpYawPitchRoll_s *var)
{
	short gyro[3], accel[3], sensors;
	unsigned char more;
  long quat[4];
	unsigned long timestamp;
//	struct dmpEuler_s sEuler;
	struct dmpGravity_s sGravity;
	struct dmpQuaternion_s sQuaternion;
	struct dmpYawPitchRoll_s sYawPitchRoll;	
	
	int rtn;
	if(mpu_set_dmp_state(1))
	{
		return 0;
	}
	do
	{
		rtn=dmp_read_fifo( gyro, accel, quat, &timestamp, &sensors, &more );
	}
	while(more);
	
	if(!rtn && (sensors & INV_WXYZ_QUAT))
	{
		
		sQuaternion.w=quat[0]/1073741824.0f;
		sQuaternion.x=quat[1]/1073741824.0f;
		sQuaternion.y=quat[2]/1073741824.0f;
		sQuaternion.z=quat[3]/1073741824.0f;

		//计算欧拉角
		//dmpGetEuler(&sEuler,&sQuaternion);
		
		//计算俯仰角
		
		dmpGetGravity(&sGravity,&sQuaternion);
		dmpGetYawPitchRoll(&sYawPitchRoll,&sQuaternion,&sGravity);
		
		var->yaw = sYawPitchRoll.yaw;
		var->pitch = sYawPitchRoll.pitch;
		var->roll = sYawPitchRoll.roll;
		
		return 1;
	}
	return 0;
}

int MPU6050_InitDMP(void)
{
	struct int_param_s int_param;

	unsigned short gyro_rate, gyro_fsr;
	unsigned char accel_fsr;
	
	int rtn;
    /* Set up MSP430 hardware. */
    rtn = MPU6050_Init();

    /* Set up gyro.
     * Every function preceded by mpu_ is a driver function and can be found
     * in inv_mpu.h.
     */
    rtn = mpu_init(&int_param);
		
		
		    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
//     result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);  //注意！！！启用外部罗盘，这里要包含 INV_XYZ_COMPASS
		rtn = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    rtn = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    rtn = mpu_set_sample_rate(100);
    /* Read back configuration in case it was set improperly. */
    rtn = mpu_get_sample_rate(&gyro_rate);
    rtn = mpu_get_gyro_fsr(&gyro_fsr);
    rtn = mpu_get_accel_fsr(&accel_fsr);

    /* Initialize HAL state variables. */
//     memset(&hal, 0, sizeof(hal));
//     hal.sensors = ACCEL_ON | GYRO_ON;
//     hal.report = PRINT_QUAT;

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    rtn = dmp_load_motion_driver_firmware();
    rtn = dmp_set_orientation(
    inv_orientation_matrix_to_scalar(gyro_orientation));
//     dmp_register_tap_cb(tap_cb);
//     dmp_register_android_orient_cb(android_orient_cb);

    rtn = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | /* DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT |*/ DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL);
    rtn = dmp_set_fifo_rate(100);
    rtn = mpu_set_dmp_state(1);
		
		//启用运动检测，第一个参数检测限值，32mg增量，第二个参数时间单位毫秒，第三个参数滤波频率
		//rtn = mpu_lp_motion_interrupt(640,1,2);
		
		MPU6050_SelfTest();

// 		mpu_set_bypass(1);

		return rtn;  //返回值按照DMP的标准，返回0不是错误。
}
//读取MPU6050 I2C Slave0 的数据
int MPU6050_Read_Ext_Sens_Data(uint8_t RegAddr,uint8_t *buff, uint8_t length)
{
//	Status rtn;
//	IIC_Buffer_Tx[0] = RegAddr;  
//	rtn = I2C_Master_BufferWrite(MPU6050_I2C, IIC_Buffer_Tx,1,Polling, MPU6050_Addr);
//	if(rtn != Success) return Error;
//	rtn = I2C_Master_BufferRead(MPU6050_I2C, buff,length,DMA, MPU6050_Addr);
//	if(rtn != Success) return Error;
//	return rtn;
	TM_I2C_ReadMulti(MPU6050_I2C, MPU6050_Addr, RegAddr, &buff[0], length);
	return 0;
}


///*以下为I2C中断处理函数*/

//__IO uint8_t Tx_Idx1=0, Rx_Idx1=0;
//extern __IO uint32_t I2CDirection;
//extern __IO uint32_t NumbOfBytes1;
//extern __IO uint8_t Address;

///**
//  * @brief  This function handles I2C1 Event interrupt request.
//  * @param  None
//  * @retval : None
//  */
//void I2C1_EV_IRQHandler(void)
//{

//    __IO uint32_t SR1Register =0;
//    __IO uint32_t SR2Register =0;
//    


//#ifdef SLAVE_DMA_USE
//    /* Read SR1 register */
//    SR1Register = I2C1->SR1;

//    /* If ADDR is set */
//    if ((SR1Register & 0x0002) == 0x0002)
//    {
//        /* In slave Transmitter/Receiver mode, when using DMA, it is recommended to update the buffer 
//          base address and the buffer size before clearing ADDR flag. In fact, the only
//          period when the slave has control  on the bus(SCL is stretched so master can not initiate 
//          transfers) is the period between ADDR is set and ADDR is cleared. Otherwise, the master can
//          initiate transfers and the buffer size & the buffer address have not yet been updated.*/

//        /* Update the DMA channels memory base address and count */
//        I2C_DMAConfig (I2C1, Buffer_Tx1, 0xFFFF, I2C_DIRECTION_TX);
//        I2C_DMAConfig (I2C1, Buffer_Rx1, 0xFFFF, I2C_DIRECTION_RX);
//        /* Clear ADDR by reading SR2 register */
//        SR2Register = I2C1->SR2;
//    }
//#else
//    /* Read the I2C1 SR1 and SR2 status registers */
//    SR1Register = I2C1->SR1;
//    SR2Register = I2C1->SR2;

//    /* If I2C1 is slave (MSL flag = 0) */
//    if ((SR2Register &0x0001) != 0x0001)
//    {
//        /* If ADDR = 1: EV1 */
//        if ((SR1Register & 0x0002) == 0x0002)
//        {
//            /* Clear SR1Register and SR2Register variables to prepare for next IT */
//            SR1Register = 0;
//            SR2Register = 0;
//            /* Initialize the transmit/receive counters for next transmission/reception
//            using Interrupt  */
//            Tx_Idx1 = 0;
//            Rx_Idx1 = 0;
//        }
//        /* If TXE = 1: EV3 */
//        if ((SR1Register & 0x0080) == 0x0080)
//        {
//            /* Write data in data register */
//            I2C1->DR = IIC_Buffer_Tx[Tx_Idx1++];
//            SR1Register = 0;
//            SR2Register = 0;
//        }
//        /* If RXNE = 1: EV2 */
//        if ((SR1Register & 0x0040) == 0x0040)
//        {
//            /* Read data from data register */
//            IIC_Buffer_Rx[Rx_Idx1++] = I2C1->DR;
//            SR1Register = 0;
//            SR2Register = 0;

//        }
//        /* If STOPF =1: EV4 (Slave has detected a STOP condition on the bus */
//        if (( SR1Register & 0x0010) == 0x0010)
//        {
//            I2C1->CR1 |= CR1_PE_Set;
//            SR1Register = 0;
//            SR2Register = 0;

//        }
//    } /* End slave mode */

//#endif

//    /* If SB = 1, I2C1 master sent a START on the bus: EV5) */
//    if ((SR1Register &0x0001) == 0x0001)
//    {

//        /* Send the slave address for transmssion or for reception (according to the configured value
//            in the write master write routine */
//        I2C1->DR = Address;
//        SR1Register = 0;
//        SR2Register = 0;
//    }
//    /* If I2C1 is Master (MSL flag = 1) */

//    if ((SR2Register &0x0001) == 0x0001)
//    {
//        /* If ADDR = 1, EV6 */
//        if ((SR1Register &0x0002) == 0x0002)
//        {
//            /* Write the first data in case the Master is Transmitter */
//            if (I2CDirection == I2C_DIRECTION_TX)
//            {
//                /* Initialize the Transmit counter */
//                Tx_Idx1 = 0;
//                /* Write the first data in the data register */
//                I2C1->DR = IIC_Buffer_Tx[Tx_Idx1++];
//                /* Decrement the number of bytes to be written */
//                NumbOfBytes1--;
//                /* If no further data to be sent, disable the I2C BUF IT
//                in order to not have a TxE  interrupt */
//                if (NumbOfBytes1 == 0)
//                {
//                    I2C1->CR2 &= (uint16_t)~I2C_IT_BUF;
//                }

//            }
//            /* Master Receiver */
//            else

//            {
//                /* Initialize Receive counter */
//                Rx_Idx1 = 0;
//                /* At this stage, ADDR is cleared because both SR1 and SR2 were read.*/
//                /* EV6_1: used for single byte reception. The ACK disable and the STOP
//                Programming should be done just after ADDR is cleared. */
//                if (NumbOfBytes1 == 1)
//                {
//                    /* Clear ACK */
//                    I2C1->CR1 &= CR1_ACK_Reset;
//                    /* Program the STOP */
//                    I2C1->CR1 |= CR1_STOP_Set;
//                }
//            }
//            SR1Register = 0;
//            SR2Register = 0;

//        }
//        /* Master transmits the remaing data: from data2 until the last one.  */
//        /* If TXE is set */
//        if ((SR1Register &0x0084) == 0x0080)
//        {
//            /* If there is still data to write */
//            if (NumbOfBytes1!=0)
//            {
//                /* Write the data in DR register */
//                I2C1->DR = IIC_Buffer_Tx[Tx_Idx1++];
//                /* Decrment the number of data to be written */
//                NumbOfBytes1--;
//                /* If  no data remains to write, disable the BUF IT in order
//                to not have again a TxE interrupt. */
//                if (NumbOfBytes1 == 0)
//                {
//                    /* Disable the BUF IT */
//                    I2C1->CR2 &= (uint16_t)~I2C_IT_BUF;
//                }
//            }
//            SR1Register = 0;
//            SR2Register = 0;
//        }
//        /* If BTF and TXE are set (EV8_2), program the STOP */
//        if ((SR1Register &0x0084) == 0x0084)
//        {

//            /* Program the STOP */
//            I2C1->CR1 |= CR1_STOP_Set;
//            /* Disable EVT IT In order to not have again a BTF IT */
//            I2C1->CR2 &= (uint16_t)~I2C_IT_EVT;
//            SR1Register = 0;
//            SR2Register = 0;
//        }
//        /* If RXNE is set */
//        if ((SR1Register &0x0040) == 0x0040)
//        {
//            /* Read the data register */
//            IIC_Buffer_Rx[Rx_Idx1++] = I2C1->DR;
//            /* Decrement the number of bytes to be read */
//            NumbOfBytes1--;
//            /* If it remains only one byte to read, disable ACK and program the STOP (EV7_1) */
//            if (NumbOfBytes1 == 1)
//            {
//                /* Clear ACK */
//                I2C1->CR1 &= CR1_ACK_Reset;
//                /* Program the STOP */
//                I2C1->CR1 |= CR1_STOP_Set;
//            }
//            SR1Register = 0;
//            SR2Register = 0;
//        }

//    }


//}



///**
//  * @brief  This function handles I2C1 Error interrupt request.
//  * @param  None
//  * @retval : None
//  */
//void I2C1_ER_IRQHandler(void)
//{

//    __IO uint32_t SR1Register =0;

//    /* Read the I2C1 status register */
//    SR1Register = I2C1->SR1;
//    /* If AF = 1 */
//    if ((SR1Register & 0x0400) == 0x0400)
//    {
//        I2C1->SR1 &= 0xFBFF;
//        SR1Register = 0;
//    }
//    /* If ARLO = 1 */
//    if ((SR1Register & 0x0200) == 0x0200)
//    {
//        I2C1->SR1 &= 0xFBFF;
//        SR1Register = 0;
//    }
//    /* If BERR = 1 */
//    if ((SR1Register & 0x0100) == 0x0100)
//    {
//        I2C1->SR1 &= 0xFEFF;
//        SR1Register = 0;
//    }

//    /* If OVR = 1 */

//    if ((SR1Register & 0x0800) == 0x0800)
//    {
//        I2C1->SR1 &= 0xF7FF;
//        SR1Register = 0;
//    }
//}
