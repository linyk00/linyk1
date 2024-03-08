#include "rtl876x_gpio.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "ltr390.h"
//#define I2C_HARD
#include <platform_utils.h>
#define GPIO_CLK       P4_0
#define GPIO_PIN_CLK         GPIO_GetPin(GPIO_CLK)
#define GPIO_SDA      P4_1
#define GPIO_PIN_SDA         GPIO_GetPin(GPIO_SDA)
//#define IIC_SCL_H app_io_write_pin(APP_IO_TYPE_NORMAL,APP_IO_PIN_12, APP_IO_PIN_SET);  //clk high

//#define IIC_SCL_L app_io_write_pin(APP_IO_TYPE_NORMAL,APP_IO_PIN_12, APP_IO_PIN_RESET);  //clk  low
#define IIC_SCL_H  GPIO_WriteBit(GPIO_PIN_CLK, (BitAction)(1));  //clk  GPIO_PIN_CLK
#define IIC_SCL_L GPIO_WriteBit(GPIO_PIN_CLK, (BitAction)(0));  //clk  low

//#define IIC_SDA_H app_io_write_pin(APP_IO_TYPE_NORMAL,APP_IO_PIN_13, APP_IO_PIN_SET);  //sda
//define IIC_SDA_L app_io_write_pin(APP_IO_TYPE_NORMAL,APP_IO_PIN_13, APP_IO_PIN_RESET);  //sda
#define IIC_SDA_H GPIO_WriteBit(GPIO_PIN_SDA, (BitAction)(1));  //sda
#define IIC_SDA_L GPIO_WriteBit(GPIO_PIN_SDA, (BitAction)(0));  //sda

uint8_t DEV_SPI_Device = 0, DEV_I2C_Device = 0;
uint8_t I2C_ADDR;

unsigned char GG_LSB=0;
unsigned char GG_MSB=0;

void CLK_OUT(void)
{
		Pad_Config(GPIO_PIN_CLK, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);

    Pinmux_Config(GPIO_PIN_CLK, DWGPIO);
}
void SDA_OUT(void)
{
		Pad_Config(GPIO_PIN_SDA, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE,
               PAD_OUT_HIGH);

    Pinmux_Config(GPIO_PIN_SDA, DWGPIO);
}
void SDA_IN(void)
{
		Pad_Config(GPIO_PIN_SDA, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);

    Pinmux_Config(GPIO_PIN_SDA, DWGPIO);
}

//???IIC
void i2c_init(void)
{	
		SDA_OUT();
		CLK_OUT();
}

//??IIC????
void IIC_Start(void)
{
	SDA_OUT();     //sda???
	IIC_SDA_H;	  	  
	IIC_SCL_H;
	platform_delay_us(4);
 	IIC_SDA_L;//START:when CLK is high,DATA change form high to low 
	platform_delay_us(4);
	IIC_SCL_L;//??I2C??,????????? 
}	  
//??IIC????
void IIC_Stop(void)
{
	SDA_OUT();//sda???
	IIC_SCL_L;
	IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	platform_delay_us(4);
	IIC_SCL_H;
	IIC_SDA_H;//??I2C??????
	platform_delay_us(4);							   	
}
//????????
//???:1,??????
//        0,??????
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA?????  
	IIC_SDA_H;
	platform_delay_us(1);	   
	IIC_SCL_H;
	platform_delay_us(1);	 
	while(GPIO_ReadInputDataBit(GPIO_PIN_SDA))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L;//????0 	   
	return 0;  
} 
//??ACK??
void IIC_Ack(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_L;
	platform_delay_us(2);
	IIC_SCL_H;
	platform_delay_us(2);
	IIC_SCL_L;
}
//???ACK??		    
void IIC_NAck(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_H;
	platform_delay_us(2);
	IIC_SCL_H;
	platform_delay_us(2);
	IIC_SCL_L;
}					 				     
//IIC??????
//????????
//1,???
//0,???			  
void IIC_Write_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL_L;//??????????
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
		{
			IIC_SDA_H;
		}
		 else
		 {
			IIC_SDA_L;
		 }
		txd<<=1; 	  
		platform_delay_us(2);   //??????????
		IIC_SCL_H;
		platform_delay_us(2); 
		IIC_SCL_L;	
		platform_delay_us(2);
    }	 
} 	    
//?1???,ack=1?,??ACK,ack=0,??nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA?????
    for(i=0;i<8;i++ )
	{
        IIC_SCL_L; 
        platform_delay_us(2);
				IIC_SCL_H;
        receive<<=1;
        if(GPIO_ReadInputDataBit(GPIO_PIN_SDA))receive++;   
		platform_delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//??nACK
    else
        IIC_Ack(); //??ACK   
    return receive;
}
void DEV_I2C_Init(uint8_t Add)
{
	DEV_I2C_Device = 1;
	I2C_ADDR =  Add;
}
uint8_t DEV_ModuleInit(void)
{
		i2c_init();
    DEV_I2C_Init(0x53 << 1);
    return 0;
}
void LTR390_Write(uint8_t DevAddress,uint8_t Register,uint8_t value)
{
	IIC_Start();
	IIC_Write_Byte(DevAddress);//
	IIC_Wait_Ack();
	IIC_Write_Byte(Register);//
	IIC_Wait_Ack();
	IIC_Write_Byte(value);//
	IIC_Wait_Ack();
	IIC_Stop();
	//delay_ms(100);
}

uint8_t LTR390_Read(uint8_t DevAddress,uint8_t Register)
{
	uint8_t Read_Temp;
	IIC_Start();
	IIC_Write_Byte(DevAddress);//
	IIC_Wait_Ack();
	IIC_Write_Byte(Register);//
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Write_Byte(DevAddress+1);//
	IIC_Wait_Ack();
	Read_Temp=IIC_Read_Byte(0);
	IIC_Stop();	
	return Read_Temp;
}

uint8_t LTR390_Init(void)
{
//	printf("LTR390 VOC Sensor Init\r\n");
	uint8_t Rdata = LTR390_Read(I2C_ADDR,LTR390_PART_ID);//ID
	if(Rdata != 0xb2) 
	{ 
		printf("ID should = 0xb2, but ID = 0X%d\r\n", Rdata);
		return 1;
	}
	printf("ID should = 0xb2, and ID = 0x%02x\r\n", Rdata);
	
	Rdata = LTR390_Read(I2C_ADDR,LTR390_MEAS_RATE);//ALS_UVS_MEAS_RATE  26143 18
	printf("ALS_UVS_MEAS_RATE = 0x%02x\r\n", Rdata);
	Rdata = LTR390_Read(I2C_ADDR,LTR390_GAIN);//ALS_UVS_GAIN
	printf("ALS_UVS_GAIN = 0x%02x\r\n", Rdata);
	LTR390_Write(I2C_ADDR,LTR390_MEAS_RATE, RESOLUTION_18BIT_TIME100MS | RATE_100MS);// default
  LTR390_Write(I2C_ADDR,LTR390_GAIN, GAIN_3); //default
	return 0;
}
uint32_t LTR390_UVS(void)
{
    LTR390_Write(I2C_ADDR,LTR390_INT_CFG, 0x34); // UVS_INT_EN=1, Command=0x34
    LTR390_Write(I2C_ADDR,LTR390_MAIN_CTRL, 0x0A); //  UVS in Active Mode
    uint32_t Data1 = LTR390_Read(I2C_ADDR,LTR390_UVSDATA);
    uint32_t Data2 = LTR390_Read(I2C_ADDR,LTR390_UVSDATA + 1);
    uint32_t Data3 = LTR390_Read(I2C_ADDR,LTR390_UVSDATA + 2);
    uint32_t uvs;
    uvs=  (Data3<<16)| (Data2<<8) | Data1;
    return uvs;
}

uint32_t LTR390_ALS(void)
{
    LTR390_Write(I2C_ADDR,LTR390_INT_CFG, 0x14); // UVS_INT_EN=1, Command=0x34
    LTR390_Write(I2C_ADDR,LTR390_MAIN_CTRL, 0x02); //  UVS in Active Mode
    uint32_t Data1 = LTR390_Read(I2C_ADDR,LTR390_ALSDATA);
    uint32_t Data2 = LTR390_Read(I2C_ADDR,LTR390_ALSDATA + 1);
    uint32_t Data3 = LTR390_Read(I2C_ADDR,LTR390_ALSDATA + 2);
    uint32_t als;
    als =  (Data3<<16)| (Data2<<8) | Data1;
    return als; 
}

void LTR390_SetIntVal(uint32_t low, uint32_t high)//LTR390_THRESH_UP and LTR390_THRESH_LOW
{
    LTR390_Write(I2C_ADDR,0x21, high&0xff);
    LTR390_Write(I2C_ADDR,0x22, (high>>8)&0xff);
    LTR390_Write(I2C_ADDR,0x23, (high>>16)&0x0f);
    LTR390_Write(I2C_ADDR,0x24, low&0xff);
    LTR390_Write(I2C_ADDR,0x25, (low>>8)&0xff);
    LTR390_Write(I2C_ADDR,0x26, (low>>16)&0x0f);
}