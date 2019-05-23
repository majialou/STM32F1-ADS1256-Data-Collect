/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

#include "ADS1255.h"


extetrn SPI_HandleTypeDef hspi2;
/**********************************************************************/
/****************************************
延时uS  
*****************************************/

void ads1255_delayus(uint16_t time)						  
{
  
}

/****************************************
功能：延时Ms
*****************************************/
void ads1255_delayms(uint16_t time)
{
  
}

/****************************************
功能：写一字节数据
*****************************************/

void ADS1255_write_bit(uint8_t temp)
{
  HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2,&temp,1,10);
  HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_SET); 
}

/****************************************
功能：读一字节数据
*****************************************/

uint8_t ADS1255_read_bit(void)
{
  uint8_t data;
  HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_RESET);
  HAL_SPI_Receive(&hspi2,&data,1,10);
  HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_SET); 
  return data; 
}

/****************************************
功能：读DRDY引脚状态
*****************************************/
uint8_t ADS1255_DRDY(void)
{
  return ADS1255_Read_DRDY;
}

/****************************************
功能：读单次数据命令
*****************************************/
void ADS1255_RDATA(void)
{
  while(ADS1255_Read_DRDY);
  ADS1255_write_bit(0x01);
}


/****************************************
功能:连续读数据命令
*****************************************/
void ADS1255_RDATAC(void)
{
  while(ADS1255_Read_DRDY);
  ADS1255_write_bit(0x03);
}

/****************************************
功能:停止连续读数据命令
*****************************************/
void ADS1255_SDATAC(void)
{
  while(ADS1255_Read_DRDY);
  ADS1255_write_bit(0x0F);
}

/****************************************
功能:补偿和增益自我校准命令
*****************************************/
void ADS1255_SELFCAL(void)
{
  while(ADS1255_Read_DRDY);
  ADS1255_write_bit(0xF0);
}


/****************************************
功能:补偿自我校准
*****************************************/
void ADS1255_SELFOCAL(void)
{
  while(ADS1255_Read_DRDY);
  ADS1255_write_bit(0xF1);
}


/****************************************
功能:增益自我校准
*****************************************/
void ADS1255_SELFGCAL(void)
{
  while(ADS1255_Read_DRDY);
  ADS1255_write_bit(0xF2);
}


/****************************************
功能:系统补偿校准
*****************************************/
void ADS1255_SYSOCAL(void)
{
  while(ADS1255_Read_DRDY);
  ADS1255_write_bit(0xF3);
}


/****************************************
功能:系统增益校准
*****************************************/
void ADS1255_SYSGCAL(void)
{
  while(ADS1255_Read_DRDY);
  ADS1255_write_bit(0xF4);
}


/****************************************
功能:AD转换同步
*****************************************/
void ADS1255_SYNC(void)
{
  while(ADS1255_Read_DRDY);
  ADS1255_write_bit(0xFC);
}


/****************************************
功能:启动待机模式
*****************************************/
void ADS1255_ATANDBY(void)
{
  ADS1255_write_bit(0xFD);
}


/****************************************
功能:系统复位
*****************************************/
void ADS1255_RESET(void)
{
  ADS1255_write_bit(0xFE);
}


/****************************************
功能:退出待机模式
*****************************************/
void ADS1255_WAKEUP(void)
{
  ADS1255_write_bit(0xFF);
}


/****************************************
功能：ADS1255写寄存器
说明：根据要求写入寄存器和命令字
*****************************************/

void ADS1255_write_reg(uint8_t ADS1255_command,uint8_t ADS1255_data)
{
  while(ADS1255_Read_DRDY); 
  ADS1255_write_bit(ADS1255_command | 0x50);
  ADS1255_write_bit(0x00);
  ADS1255_write_bit(ADS1255_data);
  ads1255_delayms(2);
}


/****************************************
功能：ADS1255读寄存器
说明：根据要求写入寄存器地址
*****************************************/

uint8_t ADS1255_read_reg(uint8_t ADS1255_command)
{
  uint8_t reg_data;
  while(ADS1255_Read_DRDY);
  ADS1255_write_bit(ADS1255_command | 0x10);
  ADS1255_write_bit(0x00);
  ads1255_delayus(50);
  reg_data=ADS1255_read_bit();
  return reg_data;
} 

/****************************************
功能：寄存器设置初始化,如果初始化成功返回0，失败返回1
*****************************************/
uint8_t ADS1255_Init(void)
{
  uint8_t ReturnData = 0;
  uint8_t ADS1255_reg_Init[5]={													
    0x04,  //状态寄存器初始化值
    0x01,  //模拟多路选择器初始化值
    0x20,  //AD控制寄存器初始化值
    0x03,  //数据速度寄存器初始化值	
    0x00, //I/O控制寄存器初始化值
  };
  ADS1255_Write_CS_H;
  ADS1255_Write_SYNC_H;
  ADS1255_Write_SCLK_L;
  ADS1255_Write_RST_L;
  ads1255_delayms(1);
  ADS1255_Write_RST_H;
  ads1255_delayms(1);
  ADS1255_Write_CS_L;	
  ads1255_delayms(1);												
  
  ADS1255_write_reg(0x00,ADS1255_reg_Init[0]);//状态寄存器初始化
  ads1255_delayus(1);
  
  ADS1255_write_reg(0x01,ADS1255_reg_Init[1]);//模拟多路选择器初始化
  ads1255_delayus(1);
  
  ADS1255_write_reg(0x02,ADS1255_reg_Init[2]);//AD控制寄存器初始化
  ads1255_delayus(1);
  
  ADS1255_write_reg(0x03,ADS1255_reg_Init[3]);//数据速度寄存器初始化	
  ads1255_delayus(1);
  
  ADS1255_write_reg(0x04,ADS1255_reg_Init[4]);//I/O控制寄存器初始化
  ads1255_delayus(1);
  
  if(ADS1255_reg_Init[1] != ADS1255_read_reg(0x01))  ReturnData = 1;
  
  if(ADS1255_reg_Init[2] != ADS1255_read_reg(0x02))  ReturnData = 1;
  ads1255_delayus(1);
  
  if(ADS1255_reg_Init[3] != ADS1255_read_reg(0x03))  ReturnData = 1;
  ads1255_delayus(1);
  
  if(ADS1255_reg_Init[4] != ADS1255_read_reg(0x04))  ReturnData = 1;
  ads1255_delayus(1);
  
  while(ADS1255_Read_DRDY);	
  ADS1255_SELFCAL();	//补偿和增益自校准
  ads1255_delayus(5);
  ADS1255_SYNC();     //AD转换同步
  ads1255_delayms(20);
  ADS1255_WAKEUP();   //退出待机模式
  ads1255_delayus(5);	
  
  return(ReturnData);
}
/****************************************
//功能:读一次转化完成的数据
*****************************************/
uint32_t ADS1255_Read_a_Data(void)
{
  uint32_t Data,Data1,Data2,Data3; 
  Data1 = ADS1255_read_bit();
  Data2 = ADS1255_read_bit();
  Data3 = ADS1255_read_bit();
  Data = (Data1<<16) | (Data2<<8) | Data3;
  return (Data);
}

/****************************************
//功能:把读数转化成电压值,输入分别为 ： 读回的二进制值   参考电压   内置增益
*****************************************/
double ADS1255_DataFormatting(uint32_t Data , double Vref ,uint8_t PGA)
{
  /*
  电压计算公式；
  设：AD采样的电压为Vin ,AD采样二进制值为X，参考电压为 Vr ,内部集成运放增益为G
  Vin = ( (2*Vr) / G ) * ( x / (2^23 -1))
  */
  double ReadVoltage;
  if(Data & 0x00800000)
  {
    Data = (~Data) & 0x00FFFFFF;
    ReadVoltage = -(((double)Data) / 8388607) * ((2*Vref) / ((double)PGA));
  }
  else
    ReadVoltage =  (((double)Data) / 8388607) * ((2*Vref) / ((double)PGA));
  
  return(ReadVoltage);
}



/*
*********************************************************************************************************
*	name: ADS1256_CfgADC
*	function: The configuration parameters of ADC, gain and data rate
*	parameter: _gain:gain 1-64
*                      _drate:  data  rate
*	The return value: NULL
*********************************************************************************************************
*/
void ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate)
{
  g_tADS1256.Gain = _gain;
  g_tADS1256.DataRate = _drate;
  
  ADS1256_WaitDRDY();
  
 
    uint8_t buf[4];		/* Storage ads1256 register configuration parameters */
    
    /*Status register define
    Bits 7-4 ID3, ID2, ID1, ID0  Factory Programmed Identification Bits (Read Only)
    
    Bit 3 ORDER: Data Output Bit Order
    0 = Most Significant Bit First (default)
    1 = Least Significant Bit First
    Input data  is always shifted in most significant byte and bit first. Output data is always shifted out most significant
    byte first. The ORDER bit only controls the bit order of the output data within the byte.
    
    Bit 2 ACAL : Auto-Calibration
    0 = Auto-Calibration Disabled (default)
    1 = Auto-Calibration Enabled
    When Auto-Calibration is enabled, self-calibration begins at the completion of the WREG command that changes
    the PGA (bits 0-2 of ADCON register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the STATUS register)
    values.
    
    Bit 1 BUFEN: Analog Input Buffer Enable
    0 = Buffer Disabled (default)
    1 = Buffer Enabled
    
    Bit 0 DRDY :  Data Ready (Read Only)
    This bit duplicates the state of the DRDY pin.
    
    ACAL=1  enable  calibration
    */
    //buf[0] = (0 << 3) | (1 << 2) | (1 << 1);//enable the internal buffer
    buf[0] = (0 << 3) | (1 << 2) | (0 << 1);  // The internal buffer is prohibited
    
    //ADS1256_WriteReg(REG_STATUS, (0 << 3) | (1 << 2) | (1 << 1));
    
    buf[1] = 0x08;	
    
    /*	ADCON: A/D Control Register (Address 02h)
    Bit 7 Reserved, always 0 (Read Only)
    Bits 6-5 CLK1, CLK0 : D0/CLKOUT Clock Out Rate Setting
    00 = Clock Out OFF
    01 = Clock Out Frequency = fCLKIN (default)
    10 = Clock Out Frequency = fCLKIN/2
    11 = Clock Out Frequency = fCLKIN/4
    When not using CLKOUT, it is recommended that it be turned off. These bits can only be reset using the RESET pin.
    
    Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources
    00 = Sensor Detect OFF (default)
    01 = Sensor Detect Current = 0.5 ?? A
    10 = Sensor Detect Current = 2 ?? A
    11 = Sensor Detect Current = 10?? A
    The Sensor Detect Current Sources can be activated to verify  the integrity of an external sensor supplying a signal to the
    ADS1255/6. A shorted sensor produces a very small signal while an open-circuit sensor produces a very large signal.
    
    Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
    000 = 1 (default)
    001 = 2
    010 = 4
    011 = 8
    100 = 16
    101 = 32
    110 = 64
    111 = 64
    */
    buf[2] = (0 << 5) | (0 << 3) | (_gain << 0);
    //ADS1256_WriteReg(REG_ADCON, (0 << 5) | (0 << 2) | (GAIN_1 << 1));	/*choose 1: gain 1 ;input 5V/
    buf[3] = s_tabDataRate[_drate];	// DRATE_10SPS;	
    
    CS_0();	/* SPI?? = 0 */
    ADS1256_Send8Bit(CMD_WREG | 0);	/* Write command register, send the register address */
    ADS1256_Send8Bit(0x03);			/* Register number 4,Initialize the number  -1*/
    
    ADS1256_Send8Bit(buf[0]);	/* Set the status register */
    ADS1256_Send8Bit(buf[1]);	/* Set the input channel parameters */
    ADS1256_Send8Bit(buf[2]);	/* Set the ADCON control register,gain */
    ADS1256_Send8Bit(buf[3]);	/* Set the output rate */
    
    CS_1();	/* SPI  cs = 1 */
  
  
 
}


/*
*********************************************************************************************************
*	name: ADS1256_DelayDATA
*	function: delay
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_DelayDATA(void)
{
  /*
  Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
  min  50   CLK = 50 * 0.13uS = 6.5uS
  */
  bsp_DelayUS(10);	/* The minimum time delay 6.5us */
}




/*
*********************************************************************************************************
*	name: ADS1256_Recive8Bit
*	function: SPI bus receive function
*	parameter: NULL
*	The return value: NULL
*********************************************************************************************************
*/
static uint8_t ADS1256_Recive8Bit(void)
{
  uint8_t read = 0;
  read = bcm2835_spi_transfer(0xff);
  return read;
}

/*
*********************************************************************************************************
*	name: ADS1256_WriteReg
*	function: Write the corresponding register
*	parameter: _RegID: register  ID
*			 _RegValue: register Value
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue)
{
  CS_0();	/* SPI  cs  = 0 */
  ADS1256_Send8Bit(CMD_WREG | _RegID);	/*Write command register */
  ADS1256_Send8Bit(0x00);		/*Write the register number */
  
  ADS1256_Send8Bit(_RegValue);	/*send register value */
  CS_1();	/* SPI   cs = 1 */
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadReg
*	function: Read  the corresponding register
*	parameter: _RegID: register  ID
*	The return value: read register value
*********************************************************************************************************
*/
static uint8_t ADS1256_ReadReg(uint8_t _RegID)
{
  uint8_t read;
  
  CS_0();	/* SPI  cs  = 0 */
  ADS1256_Send8Bit(CMD_RREG | _RegID);	/* Write command register */
  ADS1256_Send8Bit(0x00);	/* Write the register number */
  
  ADS1256_DelayDATA();	/*delay time */
  
  read = ADS1256_Recive8Bit();	/* Read the register values */
  CS_1();	/* SPI   cs  = 1 */
  
  return read;
}

/*
*********************************************************************************************************
*	name: ADS1256_WriteCmd
*	function: Sending a single byte order
*	parameter: _cmd : command
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_WriteCmd(uint8_t _cmd)
{
  CS_0();	/* SPI   cs = 0 */
  ADS1256_Send8Bit(_cmd);
  CS_1();	/* SPI  cs  = 1 */
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadChipID
*	function: Read the chip ID
*	parameter: _cmd : NULL
*	The return value: four high status register
*********************************************************************************************************
*/
uint8_t ADS1256_ReadChipID(void)
{
  uint8_t id;
  
  ADS1256_WaitDRDY();
  id = ADS1256_ReadReg(REG_STATUS);
  return (id >> 4);
}

/*
*********************************************************************************************************
*	name: ADS1256_SetChannal
*	function: Configuration channel number
*	parameter:  _ch:  channel number  0--7
*	The return value: NULL
*********************************************************************************************************
*/
static void ADS1256_SetChannal(uint8_t _ch)
{
  /*
  Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
  0000 = AIN0 (default)
  0001 = AIN1
  0010 = AIN2 (ADS1256 only)
  0011 = AIN3 (ADS1256 only)
  0100 = AIN4 (ADS1256 only)
  0101 = AIN5 (ADS1256 only)
  0110 = AIN6 (ADS1256 only)
  0111 = AIN7 (ADS1256 only)
  1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ??don??t care??)
  
  NOTE: When using an ADS1255 make sure to only select the available inputs.
  
  Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
  0000 = AIN0
  0001 = AIN1 (default)
  0010 = AIN2 (ADS1256 only)
  0011 = AIN3 (ADS1256 only)
  0100 = AIN4 (ADS1256 only)
  0101 = AIN5 (ADS1256 only)
  0110 = AIN6 (ADS1256 only)
  0111 = AIN7 (ADS1256 only)
  1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ??don??t care??)
  */
  if (_ch > 7)
  {
    return;
  }
  ADS1256_WriteReg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN connection AINCOM */
}

/*
*********************************************************************************************************
*	name: ADS1256_SetDiffChannal
*	function: The configuration difference channel
*	parameter:  _ch:  channel number  0--3
*	The return value:  four high status register
*********************************************************************************************************
*/
static void ADS1256_SetDiffChannal(uint8_t _ch)
{
  /*
  Bits 7-4 PSEL3, PSEL2, PSEL1, PSEL0: Positive Input Channel (AINP) Select
  0000 = AIN0 (default)
  0001 = AIN1
  0010 = AIN2 (ADS1256 only)
  0011 = AIN3 (ADS1256 only)
  0100 = AIN4 (ADS1256 only)
  0101 = AIN5 (ADS1256 only)
  0110 = AIN6 (ADS1256 only)
  0111 = AIN7 (ADS1256 only)
  1xxx = AINCOM (when PSEL3 = 1, PSEL2, PSEL1, PSEL0 are ??don??t care??)
  
  NOTE: When using an ADS1255 make sure to only select the available inputs.
  
  Bits 3-0 NSEL3, NSEL2, NSEL1, NSEL0: Negative Input Channel (AINN)Select
  0000 = AIN0
  0001 = AIN1 (default)
  0010 = AIN2 (ADS1256 only)
  0011 = AIN3 (ADS1256 only)
  0100 = AIN4 (ADS1256 only)
  0101 = AIN5 (ADS1256 only)
  0110 = AIN6 (ADS1256 only)
  0111 = AIN7 (ADS1256 only)
  1xxx = AINCOM (when NSEL3 = 1, NSEL2, NSEL1, NSEL0 are ??don??t care??)
  */
  if (_ch == 0)
  {
    ADS1256_WriteReg(REG_MUX, (0 << 4) | 1);	/* DiffChannal  AIN0?? AIN1 */
  }
  else if (_ch == 1)
  {
    ADS1256_WriteReg(REG_MUX, (2 << 4) | 3);	/*DiffChannal   AIN2?? AIN3 */
  }
  else if (_ch == 2)
  {
    ADS1256_WriteReg(REG_MUX, (4 << 4) | 5);	/*DiffChannal    AIN4?? AIN5 */
  }
  else if (_ch == 3)
  {
    ADS1256_WriteReg(REG_MUX, (6 << 4) | 7);	/*DiffChannal   AIN6?? AIN7 */
  }
}

/*
*********************************************************************************************************
*	name: ADS1256_WaitDRDY
*	function: delay time  wait for automatic calibration
*	parameter:  NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static void ADS1256_WaitDRDY(void)
{
  uint32_t i;
  
  for (i = 0; i < 400000; i++)
  {
    if (DRDY_IS_LOW())
    {
      break;
    }
  }
  if (i >= 400000)
  {
    printf("ADS1256_WaitDRDY() Time Out ...\r\n");		
  }
}

/*
*********************************************************************************************************
*	name: ADS1256_ReadData
*	function: read ADC value
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static int32_t ADS1256_ReadData(void)
{
  uint32_t read = 0;
  static uint8_t buf[3];
  
  CS_0();	/* SPI   cs = 0 */
  
  ADS1256_Send8Bit(CMD_RDATA);	/* read ADC command  */
  
  ADS1256_DelayDATA();	/*delay time  */
  
  /*Read the sample results 24bit*/
  buf[0] = ADS1256_Recive8Bit();
  buf[1] = ADS1256_Recive8Bit();
  buf[2] = ADS1256_Recive8Bit();
  
  read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
  read |= ((uint32_t)buf[1] << 8);  /* Pay attention to It is wrong   read |= (buf[1] << 8) */
  read |= buf[2];
  
  CS_1();	/* SPI?? = 1 */
  
  /* Extend a signed number*/
  if (read & 0x800000)
  {
    read |= 0xFF000000;
  }
  
  return (int32_t)read;
}


/*
*********************************************************************************************************
*	name: ADS1256_GetAdc
*	function: read ADC value
*	parameter:  channel number 0--7
*	The return value:  ADC vaule (signed number)
*********************************************************************************************************
*/
int32_t ADS1256_GetAdc(uint8_t _ch)
{
  int32_t iTemp;
  
  if (_ch > 7)
  {
    return 0;
  }
  
  iTemp = g_tADS1256.AdcNow[_ch];
  
  return iTemp;
}

/*
*********************************************************************************************************
*	name: ADS1256_ISR
*	function: Collection procedures
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
void ADS1256_ISR(void)
{
  if (g_tADS1256.ScanMode == 0)	/*  0  Single-ended input  8 channel?? 1 Differential input  4 channe */
  {
    
    ADS1256_SetChannal(g_tADS1256.Channel);	/*Switch channel mode */
    bsp_DelayUS(5);
    
    ADS1256_WriteCmd(CMD_SYNC);
    bsp_DelayUS(5);
    
    ADS1256_WriteCmd(CMD_WAKEUP);
    bsp_DelayUS(25);
    
    if (g_tADS1256.Channel == 0)
    {
      g_tADS1256.AdcNow[7] = ADS1256_ReadData();	
    }
    else
    {
      g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	
    }
    
    if (++g_tADS1256.Channel >= 8)
    {
      g_tADS1256.Channel = 0;
    }
  }
  else	/*DiffChannal*/
  {
    
    ADS1256_SetDiffChannal(g_tADS1256.Channel);	/* change DiffChannal */
    bsp_DelayUS(5);
    
    ADS1256_WriteCmd(CMD_SYNC);
    bsp_DelayUS(5);
    
    ADS1256_WriteCmd(CMD_WAKEUP);
    bsp_DelayUS(25);
    
    if (g_tADS1256.Channel == 0)
    {
      g_tADS1256.AdcNow[3] = ADS1256_ReadData();	
    }
    else
    {
      g_tADS1256.AdcNow[g_tADS1256.Channel-1] = ADS1256_ReadData();	
    }
    
    if (++g_tADS1256.Channel >= 4)
    {
      g_tADS1256.Channel = 0;
    }
  }
}




