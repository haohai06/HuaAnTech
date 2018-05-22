
#include "stm32f10x.h"
#include "PCF8653.h"

#define SDA_H()        (GPIO->BSRR|=SDA_Pin)
#define SDA_L()        (GPIO->BRR|=SDA_Pin)
#define SCL_H()        (GPIO->BSRR|=SCL_Pin)
#define SCL_L()        (GPIO->BRR|=SCL_Pin)

#define Write_ADD       0xA2       //PCF8563��д��ַ
#define Read_ADD        0xA3        //PCF8563�Ķ���ַ

#define PCF8563_Start()             IIC_Start(PCF8563_GPIO, PCF8563_SCL, PCF8563_SDA)
#define PCF8563_Send_Byte(n)        IIC_Send_Byte((n), PCF8563_GPIO, PCF8563_SCL, PCF8563_SDA)
#define PCF8563_Read_Byte(n)        IIC_Read_Byte((n), PCF8563_GPIO, PCF8563_SCL, PCF8563_SDA)
#define PCF8563_Wait_Ack()          IIC_Wait_Ack(PCF8563_GPIO, PCF8563_SCL, PCF8563_SDA)
#define PCF8563_Stop()              IIC_Stop(PCF8563_GPIO, PCF8563_SCL, PCF8563_SDA)

#define SECOND_ADD      0x02
#define MINUTES_ADD     0x03
#define HOUR_ADD        0x04
#define DAY_ADD         0x05
#define WEEK_ADD        0x06
#define MONTH_ADD       0x07
#define YEAR_ADD        0x08
#define CONTROL_STATUS1 0x00
#define CONTROL_STATUS2 0x01
#define CLKOUT_CONTROL  0x0D


 
//************************************
// Method:    IIC_Init
// FullName:  IIC_Init
// Access:    public 
// Returns:   void
// Parameter: GPIO_TypeDef * GPIO
// Parameter: uint16_t SCL_Pin
// Parameter: uint16_t SDA_Pin
// Description:        ��ʼ��IIC������SDA��SCLΪ��©�������ʼΪ�ߵ�ƽ
//************************************
void IIC_Init(GPIO_TypeDef * GPIO, uint16_t SCL_Pin, uint16_t SDA_Pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);        //����Ӧ���У�����IIC��������GPIOB��

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;             //��©���
	GPIO_InitStructure.GPIO_Pin = SCL_Pin | SDA_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIO, &GPIO_InitStructure);

	//GPIO->BSRR |= SDA_Pin;
	//GPIO->BRR |= SCL_Pin;
	GPIO_SetBits(GPIO, SDA_Pin | SCL_Pin);        //SDA��SCL��ʼ״̬Ϊ�ߵ�ƽ
}

//************************************
// Method:    DecToBCD
// FullName:  DecToBCD
// Access:    public 
// Returns:   unsigned char
// Parameter: unsigned char _dec
// Description:        ʮ����תBCD����
//************************************
unsigned char DecToBCD(unsigned char _dec)
{
	unsigned char temp = 0;
	while (_dec >= 10)
	{
		temp++;
		_dec -= 10;
	}
	return ((unsigned char)(temp << 4) | _dec);
}

//************************************
// Method:    BCDToDec
// FullName:  BCDToDec
// Access:    public 
// Returns:   unsigned char
// Parameter: unsigned char _BCD
// Description:        BCD����תʮ����
//************************************
unsigned char BCDToDec(unsigned char _BCD)
{
	unsigned char temp = 0;
	temp = ((unsigned char)(_BCD & (unsigned char)0xF0) >> (unsigned char)0x04) * 10;
	return (temp + (_BCD & (unsigned char)0x0F));
}
void PCF_delay_us(u32 count)
 {
	volatile u32 i=0;
	volatile unsigned char  j=0;
	for(i=0;i<count;i++)
	{
		for(j=0;j<5;j++);
	}
	 
 }
//************************************
// Method:    IIC_Start
// FullName:  IIC_Start
// Access:    public 
// Returns:   void
// Parameter: GPIO_TypeDef * GPIO
// Parameter: uint16_t SCL_Pin
// Parameter: uint16_t SDA_Pin
// Description:        ������ʼ�źţ�SCLΪ�ߵ�ƽʱ��SDA�½���
//************************************
void IIC_Start(GPIO_TypeDef * GPIO, uint16_t SCL_Pin, uint16_t SDA_Pin)
{
	SDA_H();
	SCL_H();
	PCF_delay_us(4);
	SDA_L();        //��ʼ�ź�
	PCF_delay_us(4);
	SCL_L();        //����SCL��ƽ��׼�����ͻ��������
}

//************************************
// Method:    IIC_Stop
// FullName:  IIC_Stop
// Access:    public 
// Returns:   void
// Parameter: GPIO_TypeDef * GPIO
// Parameter: uint16_t SCL_Pin
// Parameter: uint16_t SDA_Pin
// Description:        ����ֹͣ�źţ�SCLΪ�ߵ�ƽʱ��SDA������
//************************************
void IIC_Stop(GPIO_TypeDef * GPIO, uint16_t SCL_Pin, uint16_t SDA_Pin)
{
	SCL_L();
	SDA_L();
	PCF_delay_us(4);
	SCL_H();
	PCF_delay_us(4);
	SDA_H();
	PCF_delay_us(4);
}

//************************************
// Method:    IIC_Send_Byte
// FullName:  IIC_Send_Byte
// Access:    public 
// Returns:   void
// Parameter: u8 txd
// Parameter: GPIO_TypeDef * GPIO
// Parameter: uint16_t SCL_Pin
// Parameter: uint16_t SDA_Pin
// Description:        IIC����һ���ֽ�
//************************************
void IIC_Send_Byte(u8 txd, GPIO_TypeDef * GPIO, uint16_t SCL_Pin, uint16_t SDA_Pin)
{
	u8 t;
	SCL_L();        //����ʱ�ӿ�ʼ���ݴ���
	for (t=0;t<8;t++)
	{
		GPIO_WriteBit(GPIO, SDA_Pin, (BitAction)((txd & 0x80) >> 7));
		txd <<= 1;
		PCF_delay_us(2);
		SCL_H();
		PCF_delay_us(2);
		SCL_L();
		PCF_delay_us(2);
	}
}
//************************************
// Method:    IIC_Ack
// FullName:  IIC_Ack
// Access:    public 
// Returns:   void
// Parameter: GPIO_TypeDef * GPIO
// Parameter: uint16_t SCL_Pin
// Parameter: uint16_t SDA_Pin
// Description:        ����ACKӦ���豸�ڽ��ջ��͵�һ���ֽں�������SDA��ƽ
//************************************
void IIC_Ack(GPIO_TypeDef * GPIO, uint16_t SCL_Pin, uint16_t SDA_Pin)
{
	SCL_L();
	SDA_L();
	PCF_delay_us(2);
	SCL_H();
	PCF_delay_us(2);
	SCL_L();
}
//************************************
// Method:    IIC_NAck
// FullName:  IIC_NAck
// Access:    public 
// Returns:   void
// Parameter: GPIO_TypeDef * GPIO
// Parameter: uint16_t SCL_Pin
// Parameter: uint16_t SDA_Pin
// Description:        ����NACKӦ���豸�ڽ��ջ��͵�һ���ֽں�������SDA��ƽ
//************************************
void IIC_NAck(GPIO_TypeDef * GPIO, uint16_t SCL_Pin, uint16_t SDA_Pin)
{
	SCL_L();
	SDA_H();
	PCF_delay_us(2);
	SCL_H();
	PCF_delay_us(2);
	SCL_L();
}
//************************************
// Method:    IIC_Read_Byte
// FullName:  IIC_Read_Byte
// Access:    public 
// Returns:   u8
// Parameter: unsigned char ack
//            ack=1������ACK
//            ack=0������NACK
// Parameter: GPIO_TypeDef * GPIO
// Parameter: uint16_t SCL_Pin
// Parameter: uint16_t SDA_Pin
// Description:        ��һ���ֽڣ�SCL�͵�ƽʱ���ı�SDA
//************************************
u8 IIC_Read_Byte(unsigned char ack, GPIO_TypeDef * GPIO, uint16_t SCL_Pin, uint16_t SDA_Pin)
{
	unsigned char i, receive = 0;
	SDA_H();        //�ͷ�����
	for (i = 0; i < 8; i++)
	{
		SCL_L();
		PCF_delay_us(2);
		SCL_H();
		receive <<= 1;
		if (GPIO_ReadInputDataBit(GPIO, SDA_Pin))
		{
			receive++;
		}
		PCF_delay_us(1);
	}
	if (!ack)
		IIC_NAck(GPIO, SCL_Pin, SDA_Pin);//����nACK
	else
		IIC_Ack(GPIO, SCL_Pin, SDA_Pin); //����ACK   
	return receive;
}

//************************************
// Method:    IIC_Wait_Ack
// FullName:  IIC_Wait_Ack
// Access:    public 
// Returns:   u8        1-����Ӧ��ʧ�ܣ�0-����Ӧ��ɹ�
// Parameter: GPIO_TypeDef * GPIO
// Parameter: uint16_t SCL_Pin
// Parameter: uint16_t SDA_Pin
// Description:        �ȴ�Ӧ���źŵ���
//************************************
u8 IIC_Wait_Ack(GPIO_TypeDef * GPIO, uint16_t SCL_Pin, uint16_t SDA_Pin)
{
	u8 ucErrTime = 0;
	SDA_H();                //�ͷ�����
	PCF_delay_us(1);
	SCL_H();
	PCF_delay_us(1);

	while (READ_BIT(GPIO->IDR,SDA_Pin))
	{
		ucErrTime++;
		if (ucErrTime > 250)
		{
			IIC_Stop(GPIO, SCL_Pin, SDA_Pin);
			return 1;
		}
	}
	SCL_L();   
	return 0;
}

//************************************
// Method:    PCF8563_Set
// FullName:  PCF8563_Set
// Access:    public 
// Returns:   void
// Parameter: PCF8563_REGISTER regaddr
// Parameter: unsigned char _data
// Description:        ���üĴ�����ֵ
//************************************
void PCF8563_Set(unsigned char regaddr, unsigned char _data)
{
	PCF8563_Start();
	PCF8563_Send_Byte(Write_ADD);
	PCF8563_Wait_Ack();
	PCF8563_Send_Byte(regaddr);
	PCF8563_Wait_Ack();
	PCF8563_Send_Byte(_data);
	PCF8563_Wait_Ack();
	PCF8563_Stop();
}


//************************************
// Method:    PCF8563_Init
// FullName:  PCF8563_Init
// Access:    public 
// Returns:   void
// Parameter: void
// Description:        ��ʼ��PCF8563��IO�ںͼĴ���
//************************************
void PCF8563_Init(void)
{
	IIC_Init(PCF8563_GPIO, PCF8563_SCL, PCF8563_SDA);
	PCF_delay_us(200);
	PCF_delay_us(200);
	PCF8563_Set(CONTROL_STATUS1, 0x00);
	PCF8563_Set(CONTROL_STATUS2, 0x00);
	PCF8563_Set(CLKOUT_CONTROL, 0x81);                //ʹ��CLK�����1024Hz
}

//************************************
// Method:    PCF8563_Read
// FullName:  PCF8563_Read
// Access:    public 
// Returns:   void
// Parameter: PCF8563 * pcf8563
// Description:        ��ʱ�� �ꡢ�¡��ա�ʱ����
//************************************
void PCF8563_Read(PCF8653TypeDef * pcf8563)
{
	PCF8563_Start();
	PCF8563_Send_Byte(Write_ADD);
	PCF8563_Wait_Ack();
	
	PCF8563_Send_Byte(SECOND_ADD );                //seconds 0x02	
	PCF8563_Wait_Ack();
	
	PCF8563_Start();
	PCF8563_Send_Byte(Read_ADD);
	PCF8563_Wait_Ack();
	
	pcf8563->ucSecond = PCF8563_Read_Byte(1) & 0X7F;	
	pcf8563->ucMinute = PCF8563_Read_Byte(1) & 0X7F;	
	pcf8563->ucHour = PCF8563_Read_Byte(1) & 0X3F;	
	pcf8563->ucDay = PCF8563_Read_Byte(1) & 0X3F;	
	pcf8563->ucWeek = PCF8563_Read_Byte(1);                //��ȡ���ڼĴ����������ݲ�����	
	pcf8563->ucMonth = PCF8563_Read_Byte(1) & 0X1F;	
	pcf8563->ucYear = PCF8563_Read_Byte(0);

	PCF8563_Stop();                                        //����ֹͣ�ź�

	//��ʱ���BCD������ʮ����
	pcf8563->ucSecond = BCDToDec(pcf8563->ucSecond);
	pcf8563->ucMinute = BCDToDec(pcf8563->ucMinute);
	pcf8563->ucHour = BCDToDec(pcf8563->ucHour);
	pcf8563->ucDay = BCDToDec(pcf8563->ucDay);
	pcf8563->ucMonth = BCDToDec(pcf8563->ucMonth);
	pcf8563->ucYear = BCDToDec(pcf8563->ucYear);
}

//************************************
// Method:    PCF8563_Write
// FullName:  PCF8563_Write
// Access:    public 
// Returns:   void
// Parameter: PCF8563 * pcf8563
// Description:        ��ʱ�䣨�ꡢ�¡��ա�ʱ���֣�д��PCF8563
//************************************
void PCF8563_Write(PCF8653TypeDef * pcf8563)
{
	//��ʱ���ʮ���Ʊ��BCD����	
	pcf8563->ucMinute = DecToBCD(pcf8563->ucMinute);
	pcf8563->ucHour = DecToBCD(pcf8563->ucHour);
	pcf8563->ucDay = DecToBCD(pcf8563->ucDay);
	pcf8563->ucMonth = DecToBCD(pcf8563->ucMonth);
	pcf8563->ucYear = DecToBCD(pcf8563->ucYear);	

	PCF8563_Start();
	PCF8563_Send_Byte(Write_ADD);
	PCF8563_Wait_Ack();
	PCF8563_Send_Byte(MINUTES_ADD);              //�ּĴ�����ַ0x03
	PCF8563_Wait_Ack();
	PCF8563_Send_Byte(pcf8563->ucMinute);     //���ͷ�
	PCF8563_Wait_Ack();
	PCF8563_Send_Byte(pcf8563->ucHour);       //����ʱ
	PCF8563_Wait_Ack();
	PCF8563_Send_Byte(pcf8563->ucDay);        //������
	PCF8563_Wait_Ack();
	PCF8563_Send_Byte(0x00);                 //�������ڣ�����ע���ڼ�
	PCF8563_Wait_Ack();
	PCF8563_Send_Byte(pcf8563->ucMonth);        //������
	PCF8563_Wait_Ack();
	PCF8563_Send_Byte(pcf8563->ucYear);         //������
	PCF8563_Wait_Ack();
	PCF8563_Stop();                            //����ֹͣ�ź�
}











