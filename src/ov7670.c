#include "ov7670.h"
#include "led.h"

#define I2C_MODE 1 //0:HARD; 1:SOFT

#define OV7670_DEVICE_WRITE_ADDRESS 0x42

I2C_HandleTypeDef I2C2_Handle;

extern void HAL_Delay_10ms(__IO uint32_t Delay);

void MCO_Init(void)
{
	/*__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_8;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Alternate = GPIO_AF0_MCO;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);*/
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_3);
}

void SCCB_SID_OUT(void)//����SCCB_SIDΪ���
{
  	GPIO_InitTypeDef  GPIO_InitStructure;

  	GPIO_InitStructure.Pin = GPIO_PIN_11;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  	GPIO_InitStructure.Pull = GPIO_PULLUP;
  	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void SCCB_SID_IN(void)//����SCCB_SIDΪ����
{
  	GPIO_InitTypeDef  GPIO_InitStructure;

  	GPIO_InitStructure.Pin = GPIO_PIN_11;
  	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  	GPIO_InitStructure.Pull = GPIO_PULLUP;
  	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void SCCB_Start(void)
{
    SCCB_SID_H();     //�����߸ߵ�ƽ
    HAL_Delay_10ms(5);
    SCCB_SIC_H();	   //��ʱ���߸ߵ�ʱ���������ɸ�����
    HAL_Delay_10ms(5);
    SCCB_SID_L();
    HAL_Delay_10ms(5);
    SCCB_SIC_L();	 //�����߻ָ��͵�ƽ��������������Ҫ
    HAL_Delay_10ms(5);
}

void SCCB_Stop(void)
{
    SCCB_SID_L();
    HAL_Delay_10ms(5);
    SCCB_SIC_H();
    HAL_Delay_10ms(5);
    SCCB_SID_H();
    HAL_Delay_10ms(5);
}

void noAck(void)
{
	SCCB_SID_H();
	HAL_Delay_10ms(5);
	SCCB_SIC_H();
	HAL_Delay_10ms(5);
	SCCB_SIC_L();
	HAL_Delay_10ms(5);
	SCCB_SID_L();
	HAL_Delay_10ms(5);
}

uint8_t SCCB_Write(uint8_t m_data)
{
	uint8_t j,tem;

	for(j=0;j<8;j++) //ѭ��8�η�������
	{
		if((m_data<<j)&0x80)SCCB_SID_H();
		else SCCB_SID_L();
		HAL_Delay_10ms(5);
		SCCB_SIC_H();
		HAL_Delay_10ms(5);
		SCCB_SIC_L();
		HAL_Delay_10ms(5);
	}
	HAL_Delay_10ms(1);
	SCCB_DATA_IN;
	HAL_Delay_10ms(5);
	SCCB_SIC_H();
	HAL_Delay_10ms(1);
	if(SCCB_SID_STATE)tem=0;//SDA=1����ʧ��
	else tem=1;//SDA=0���ͳɹ�������1
	SCCB_SIC_L();
	HAL_Delay_10ms(5);
    SCCB_DATA_OUT;
	return tem;
}

uint8_t SCCB_Read(void)
{
	uint8_t read,j;
	read=0x00;

	SCCB_DATA_IN;
	HAL_Delay_10ms(5);
	for(j=8;j>0;j--) //ѭ��8�ν�������
	{
		HAL_Delay_10ms(5);
		SCCB_SIC_H();
		HAL_Delay_10ms(5);
		read=read<<1;
		if(SCCB_SID_STATE)read=read+1;
		SCCB_SIC_L();
		HAL_Delay_10ms(5);
	}
    SCCB_DATA_OUT;
	return read;
}

uint8_t OV7670_Init(void)
{
	uint8_t i;

	MCO_Init();
	SCCB_Init();

	while(0x73 != OV_ReadID())
	{
		HAL_Delay(100);
		LEDToggle(0);
	}

	OV_Reset();
	HAL_Delay(5);

  	for(i=0; i<OV7670_REG_NUM; i++)
  	{
    	if(OV7670_WriteReg(OV7670_reg[i][0],OV7670_reg[i][1]))return 1;
  	}
	return 0; 
}

void SCCB_Init(void)
{
#if I2C_MODE
	GPIO_InitTypeDef  GPIO_InitStructure;
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
#else
	GPIO_InitTypeDef GPIO_InitStructure;

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure I2C2 GPIOs */
	GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Set the I2C structure parameters */
	I2C2_Handle.Instance = I2C2;

	/* I2C DeInit */
	HAL_I2C_DeInit(&I2C2_Handle);
	I2C2_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2C2_Handle.Init.OwnAddress1 = 0xFE;
	I2C2_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2C2_Handle.Init.ClockSpeed = 100000;
	HAL_I2C_Init(&I2C2_Handle);
#endif
}

uint8_t OV7670_WriteReg(uint8_t Addr, uint8_t Data)
{
#if I2C_MODE
	SCCB_Start();//����SCCB ���߿�ʼ��������
	if(SCCB_Write(OV7670_DEVICE_WRITE_ADDRESS)==0)//д��ַ
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 1;//���󷵻�
	}
	HAL_Delay_10ms(1);
	if(SCCB_Write(Addr)==0)//������ID
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 2;//���󷵻�
	}
	HAL_Delay_10ms(1);
	if(SCCB_Write(Data)==0)//д���ݵ�������
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 3;//���󷵻�
	}
	SCCB_Stop();//����SCCB ����ֹͣ��������
	return 0;//�ɹ�����
#else
	uint8_t sta = 0;
	uint8_t data[2] = {Addr, Data};
	sta = HAL_I2C_Master_Transmit(&I2C2_Handle, OV7670_DEVICE_WRITE_ADDRESS, data, 2, OV_TIMEOUT_MAX);
	if(sta)
		return sta;
	/* If operation is OK, return 0 */
	return 0;
#endif
}

uint8_t OV7670_ReadReg(uint8_t Addr)
{
#if I2C_MODE
	uint8_t Data = 0;
	//ͨ��д�������üĴ�����ַ
	SCCB_Start();
	if(SCCB_Write(OV7670_DEVICE_WRITE_ADDRESS)==0)//д��ַ
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 1;//���󷵻�
	}
	HAL_Delay_10ms(1);
	if(SCCB_Write(Addr)==0)//������ID
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 2;//���󷵻�
	}
	SCCB_Stop();//����SCCB ����ֹͣ��������
	HAL_Delay_10ms(1);
		//���üĴ�����ַ�󣬲��Ƕ�
	SCCB_Start();
	if(SCCB_Write(OV7670_DEVICE_WRITE_ADDRESS+1)==0)//����ַ
	{
		SCCB_Stop();//����SCCB ����ֹͣ��������
		return 3;//���󷵻�
	}
	HAL_Delay_10ms(1);
	Data=SCCB_Read();//���ض�����ֵ
	noAck();//����NACK����
	SCCB_Stop();//����SCCB ����ֹͣ��������
	return Data;
#else
  uint8_t Data = 0;
  if( HAL_I2C_Master_Transmit(&I2C2_Handle, OV7670_DEVICE_WRITE_ADDRESS, &Addr, 1, OV_TIMEOUT_MAX))
	  return 0xff;

  if( HAL_I2C_Master_Receive(&I2C2_Handle, OV7670_DEVICE_WRITE_ADDRESS, &Data, 1, OV_TIMEOUT_MAX) )
	  return 0xff;

  /* return the read data */
  return Data;
#endif
}

void OV_Reset(void)
{
	OV7670_WriteReg(0x12,0x80);
}

uint8_t OV_ReadID(void)
{
	uint8_t temp;
	temp = OV7670_ReadReg(0x0b);
  	return temp;
}
