#ifndef __DAC8563_H_H
#define __DAC8563_H_H
/********************************************************************/
#include "stm32f10x_GPIO.h"
#include "stm32f10x_rcc.h"
/*****************************************************************************
                        Function Declare                                  
*****************************************************************************/
/* gain channel? */
typedef enum
{
	ADS1256_GAIN_1			= (0),	/* GAIN   1 */
	ADS1256_GAIN_2			= (1),	/*GAIN   2 */
	ADS1256_GAIN_4			= (2),	/*GAIN   4 */
	ADS1256_GAIN_8			= (3),	/*GAIN   8 */
	ADS1256_GAIN_16			= (4),	/* GAIN  16 */
	ADS1256_GAIN_32			= (5),	/*GAIN    32 */
	ADS1256_GAIN_64			= (6),	/*GAIN    64 */
}ADS1256_GAIN_E;

/* Sampling speed choice*/
/* 
	11110000 = 30,000SPS (default)
	11100000 = 15,000SPS
	11010000 = 7,500SPS
	11000000 = 3,750SPS
	10110000 = 2,000SPS
	10100001 = 1,000SPS
	10010010 = 500SPS
	10000010 = 100SPS
	01110010 = 60SPS
	01100011 = 50SPS
	01010011 = 30SPS
	01000011 = 25SPS
	00110011 = 15SPS
	00100011 = 10SPS
	00010011 = 5SPS
	00000011 = 2.5SPS
*/
typedef enum
{
	ADS1256_30000SPS = 0,
	ADS1256_15000SPS,
	ADS1256_7500SPS,
	ADS1256_3750SPS,
	ADS1256_2000SPS,
	ADS1256_1000SPS,
	ADS1256_500SPS,
	ADS1256_100SPS,
	ADS1256_60SPS,
	ADS1256_50SPS,
	ADS1256_30SPS,
	ADS1256_25SPS,
	ADS1256_15SPS,
	ADS1256_10SPS,
	ADS1256_5SPS,
	ADS1256_2SPS,
	ADS1256_DRATE_MAX
}ADS1256_DRATE_E;

#define ADS1256_DRAE_COUNT = 15;





/*Register definition?? Table 23. Register Map --- ADS1256 datasheet Page 30*/
enum
{
	/*Register address, followed by reset the default values */
	REG_STATUS = 0,	// x1H
	REG_MUX    = 1, // 01H
	REG_ADCON  = 2, // 20H
	REG_DRATE  = 3, // F0H
	REG_IO     = 4, // E0H
	REG_OFC0   = 5, // xxH
	REG_OFC1   = 6, // xxH
	REG_OFC2   = 7, // xxH
	REG_FSC0   = 8, // xxH
	REG_FSC1   = 9, // xxH
	REG_FSC2   = 10, // xxH
};

/* Command definition?? TTable 24. Command Definitions --- ADS1256 datasheet Page 34 */
enum
{
	CMD_WAKEUP  = 0x00,	// Completes SYNC and Exits Standby Mode 0000  0000 (00h)
	CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
	CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
	CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
	CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
	CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
	CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
	CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
	CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
	CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
	CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
	CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
	CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
	CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};


ADS1256_VAR_T g_tADS1256;
static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] =
{
	0xF0,		/*reset the default values  */
	0xE0,
	0xD0,
	0xC0,
	0xB0,
	0xA1,
	0x92,
	0x82,
	0x72,
	0x63,
	0x53,
	0x43,
	0x33,
	0x20,
	0x13,
	0x03
};





void ads1255_delayus(uint16_t time);//功能:软件粗略延时1us
void ads1255_delayms(uint16_t time);//功能:软件粗略延时1ms

uint8_t ADS1255_Init(void);         //功能:寄存器设置初始化,如果初始化成功返回0，失败返回1
uint32_t ADS1255_Read_a_Data(void); //功能:读一次24位转化数据
uint32_t ADS1255_Read_a_Data(void); //功能:读一次转化完成的数据

double ADS1255_DataFormatting(uint32_t Data , double Vref ,uint8_t PGA); 
///功能:把读数转化成电压值,输入分别为 ： 读回的二进制值   参考电压   内置增益

uint8_t ADS1255_DRDY(void);         //功能：读DRDY引脚状态
void ADS1255_write_reg(uint8_t ADS1255_command,uint8_t ADS1255_data);//功能:向ADS1256中对应寄存器地址写一字节数据
uint8_t ADS1255_read_reg(uint8_t ADS1255_command);                   //功能:向ADS1256中对应寄存器读一字节数据

void ADS1255_write_bit(uint8_t temp);//功能:写一字节数据
uint8_t ADS1255_read_bit(void);      //功能:读一字节数据

//---------------命令函数-----------------------------//
void ADS1255_RDATA(void);    //功能:读单次数据命令
void ADS1255_RDATAC(void);   //功能:连续读数据命令
void ADS1255_SDATAC(void);   //功能:停止连续读数据命令
void ADS1255_SELFCAL(void);  //功能:补偿和增益自我校准命令
void ADS1255_SELFOCAL(void); //功能:补偿自我校准
void ADS1255_SELFGCAL(void); //功能:增益自我校准
void ADS1255_SYSOCAL(void);  //功能:系统补偿校准
void ADS1255_SYSGCAL(void);  //功能:系统增益校准
void ADS1255_SYNC(void);     //功能:AD转换同步
void ADS1255_ATANDBY(void);  //功能:启动待机模式
void ADS1255_RESET(void);    //功能:系统复位
void ADS1255_WAKEUP(void);   //功能:退出待机模式

//-------------------------------------------------------------------------------//
#endif
