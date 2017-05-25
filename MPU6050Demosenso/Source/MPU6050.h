/**************************************************************************************************
  Filename:       MPU6050.h
  
**************************************************************************************************/

#ifndef MPU6050_H
#define MPU6050_H

/******************************************************************************
 * INCLUDES
 */
/******************************************************************************
 * CONSTANTS
 */



//博创管脚定义
#define  SCL      P1_3
#define  SCL_IN   P1DIR &= ~0x08
#define  SCL_OUT  P1DIR |= 0x08
#define  SDA      P1_2
#define  SDA_IN   P1DIR &= ~0x04
#define  SDA_OUT  P1DIR |=  0x04






//MPU6050定义
#define        SMPLRT_DIV                 0x19        //陀螺仪采样率，典型值：0x07(125Hz)
#define        CONFIG                     0x1A        //低通滤波频率，典型值：0x06(5Hz)
#define        GYRO_CONFIG                0x1B        //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define        ACCEL_CONFIG               0x1C        //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define        ACCEL_XOUT_H               0x3B
#define        ACCEL_XOUT_L               0x3C
#define        ACCEL_YOUT_H               0x3D
#define        ACCEL_YOUT_L               0x3E
#define        ACCEL_ZOUT_H               0x3F
#define        ACCEL_ZOUT_L               0x40
#define        TEMP_OUT_H                 0x41
#define        TEMP_OUT_L                 0x42
#define        GYRO_XOUT_H                0x43
#define        GYRO_XOUT_L                0x44        
#define        GYRO_YOUT_H                0x45
#define        GYRO_YOUT_L                0x46
#define        GYRO_ZOUT_H                0x47
#define        GYRO_ZOUT_L                0x48
#define        PWR_MGMT_1                 0x6B        //电源管理，典型值：0x00(正常启用)
#define        WHO_AM_I                   0x75        //IIC地址寄存器(默认数值0x68，只读)
#define        MPU6050_SA_W               0xD0        //IIC写入时的地址字节数据，+1为读取
#define        MPU6050_SA_R               0xD1
#define	       SlaveAddress	          0xD0	      //IIC写入时的地址字节数据，+1为读取

//定义偏移量
#define ACCEL_XOUT_H_offset (343)
#define ACCEL_YOUT_H_offset (-154)
#define ACCEL_ZOUT_H_offset (558)

#define GYRO_XOUT_H_offset  (-14)
#define GYRO_YOUT_H_offset  (21)
#define GYRO_ZOUT_H_offset  (1)






extern void   Start(void);
extern void   Stop(void);
extern void   Port_Init(void);
extern void   SendACK(uint8 ack);
extern uint8  RecvACK(void);
extern void   SendByte(uint8 dat);
extern uint8  RecvByte(void);
extern void   Single_Write(uint8 REG_Address,uint8 REG_data);
extern uint8  Single_Read(uint8 REG_Address);
extern void   lcd_printf(uint8 *s,int16 temp_data);
extern void   InitMPU6050(void);
extern int16  GetData(uint8 REG_Address);
uint8 dis[6];
void NOP(void);
uint8 FO;
//延时5微妙
void NOP(void)
{
  uint8 i=5;
  while(i)
  {
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    asm("NOP");asm("NOP");asm("NOP");asm("NOP");
    i--;
  }
}
//**************************************
//I2C起始信号
//**************************************
void Start(void)
{
  SCL_OUT;
  SDA_OUT;
  SDA=1;
  SCL=1;
  NOP();
  SDA=0;
  NOP();
  SCL=0;
}
//**************************************
//I2C停止信号
//**************************************
void Stop(void)
{
  SCL_OUT;
  SDA_OUT;
  SDA=0;
  SCL=1;
  NOP();
  SDA=1;
  NOP();
}
//**************************************
//I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
void SendACK(uint8 ack)
{
  SCL_OUT;
  SDA_OUT;
  SDA=ack;
  SCL =1;
  NOP();
  SCL=0;
  NOP();
}
//**************************************
//I2C接收应答信号
//**************************************
uint8 RecvACK(void)
{
  
  SCL_OUT;
  SDA_IN;
  SCL=1;
  NOP();
  FO=SDA;
  SCL=0;
  NOP();
  return FO;
}
//**************************************
//向I2C总线发送一个字节数据
//**************************************
void SendByte(uint8 dat)
{
  uint8 i;
  SCL_OUT;
  SDA_OUT;
  for(i=0;i<8;i++)
  {
    if(dat&0x80)
    {
      SDA=1;
    }else
    {
      SDA=0;
    }
    dat<<=1;
    SCL=1;
    NOP();
    SCL=0;
    NOP();
  }
  RecvACK();
}
//**************************************
//从I2C总线接收一个字节数据
//**************************************
uint8 RecvByte(void)
{
  uint8 i;
  uint8 dat=0;
  SDA_IN;
  SCL_OUT;
  SDA=1;
  for(i=0;i<8;i++)
  {
    dat<<=1;
    SCL =1;
    NOP();
    dat |=SDA;
    SCL =0;
    NOP();
  }
  return dat;
}
//**************************************
//向I2C设备写入一个字节数据
//**************************************
void Single_Write(uint8 REG_Address,uint8 REG_data)
{
  Start();                  //起始信号
  SendByte(SlaveAddress);   //发送设备地址+写信号
  SendByte(REG_Address);    //内部寄存器地址
  SendByte(REG_data);       //内部寄存器数据，
  Stop();                   //发送停止信号
}
//**************************************
//从I2C设备读取一个字节数据
//**************************************
uint8 Single_Read(uint8 REG_Address)
{
  uint8 REG_data;
  Start();                   //起始信号
  SendByte(SlaveAddress);    //发送设备地址+写信号
  SendByte(REG_Address);     //发送存储单元地址，从0开始
  Start();                   //起始信号
  SendByte(SlaveAddress+1);  //发送设备地址+读信号
  REG_data=RecvByte();       //读出寄存器数据
  SendACK(1);                //接收应答信号
  Stop();                    //停止信号
  return REG_data;
}

//**************************************
//初始化MPU6050
//**************************************
void InitMPU6050(void)
{
  SCL_OUT;
  Single_Write(PWR_MGMT_1, 0x00);	//解除休眠状态
  Single_Write(SMPLRT_DIV, 0x07);
  Single_Write(CONFIG, 0x06);
  Single_Write(GYRO_CONFIG, 0x18);
  Single_Write(ACCEL_CONFIG, 0x01);
}
//**************************************
//合成数据
//**************************************
int16 GetData(uint8 REG_Address)
{
	uint8 H,L;
	H=Single_Read(REG_Address);
	L=Single_Read(REG_Address+1);
	return (H<<8)+L;   //合成数据
}
//****************************************
//整数转字符串
//****************************************
void lcd_printf(uint8 *s,int16 temp_data)
{
	if(temp_data<0)
	{
		temp_data=-temp_data;
		*s='-';
	}
	else *s=' ';

	*++s =temp_data/10000+0x30;
	temp_data=temp_data%10000;     //取余运算

	*++s =temp_data/1000+0x30;
	temp_data=temp_data%1000;     //取余运算

	*++s =temp_data/100+0x30;
	temp_data=temp_data%100;     //取余运算
	*++s =temp_data/10+0x30;
	temp_data=temp_data%10;      //取余运算
	*++s =temp_data+0x30; 	
}

#endif