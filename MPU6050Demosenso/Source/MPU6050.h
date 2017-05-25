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



//�����ܽŶ���
#define  SCL      P1_3
#define  SCL_IN   P1DIR &= ~0x08
#define  SCL_OUT  P1DIR |= 0x08
#define  SDA      P1_2
#define  SDA_IN   P1DIR &= ~0x04
#define  SDA_OUT  P1DIR |=  0x04






//MPU6050����
#define        SMPLRT_DIV                 0x19        //�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define        CONFIG                     0x1A        //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define        GYRO_CONFIG                0x1B        //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define        ACCEL_CONFIG               0x1C        //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
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
#define        PWR_MGMT_1                 0x6B        //��Դ��������ֵ��0x00(��������)
#define        WHO_AM_I                   0x75        //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define        MPU6050_SA_W               0xD0        //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ
#define        MPU6050_SA_R               0xD1
#define	       SlaveAddress	          0xD0	      //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ

//����ƫ����
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
//��ʱ5΢��
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
//I2C��ʼ�ź�
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
//I2Cֹͣ�ź�
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
//I2C����Ӧ���ź�
//��ڲ���:ack (0:ACK 1:NAK)
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
//I2C����Ӧ���ź�
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
//��I2C���߷���һ���ֽ�����
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
//��I2C���߽���һ���ֽ�����
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
//��I2C�豸д��һ���ֽ�����
//**************************************
void Single_Write(uint8 REG_Address,uint8 REG_data)
{
  Start();                  //��ʼ�ź�
  SendByte(SlaveAddress);   //�����豸��ַ+д�ź�
  SendByte(REG_Address);    //�ڲ��Ĵ�����ַ
  SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
  Stop();                   //����ֹͣ�ź�
}
//**************************************
//��I2C�豸��ȡһ���ֽ�����
//**************************************
uint8 Single_Read(uint8 REG_Address)
{
  uint8 REG_data;
  Start();                   //��ʼ�ź�
  SendByte(SlaveAddress);    //�����豸��ַ+д�ź�
  SendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ
  Start();                   //��ʼ�ź�
  SendByte(SlaveAddress+1);  //�����豸��ַ+���ź�
  REG_data=RecvByte();       //�����Ĵ�������
  SendACK(1);                //����Ӧ���ź�
  Stop();                    //ֹͣ�ź�
  return REG_data;
}

//**************************************
//��ʼ��MPU6050
//**************************************
void InitMPU6050(void)
{
  SCL_OUT;
  Single_Write(PWR_MGMT_1, 0x00);	//�������״̬
  Single_Write(SMPLRT_DIV, 0x07);
  Single_Write(CONFIG, 0x06);
  Single_Write(GYRO_CONFIG, 0x18);
  Single_Write(ACCEL_CONFIG, 0x01);
}
//**************************************
//�ϳ�����
//**************************************
int16 GetData(uint8 REG_Address)
{
	uint8 H,L;
	H=Single_Read(REG_Address);
	L=Single_Read(REG_Address+1);
	return (H<<8)+L;   //�ϳ�����
}
//****************************************
//����ת�ַ���
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
	temp_data=temp_data%10000;     //ȡ������

	*++s =temp_data/1000+0x30;
	temp_data=temp_data%1000;     //ȡ������

	*++s =temp_data/100+0x30;
	temp_data=temp_data%100;     //ȡ������
	*++s =temp_data/10+0x30;
	temp_data=temp_data%10;      //ȡ������
	*++s =temp_data+0x30; 	
}

#endif