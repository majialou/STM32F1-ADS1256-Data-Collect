/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADS1256_H
#define __ADS1256_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"


#define ADS1256_SIGNGLE_INPUT         0
#define ADS1256_DIFFERENTIAL_INPUT    1 

 /*  0  Single-ended input  8 channel?? 1 Differential input  4 channe */
#define ADC1256_INPUT_MODE   ADS1256_SIGNGLE_INPUT        //Single-ended input 
//#define ADC1256_INPUT_MODE   ADS1256_DIFFERENTIAL_INPUT  //ADifferential input

#if (ADC1256_INPUT_MODE == ADS1256_SIGNGLE_INPUT)
#define ADS1256_CHANNEL_NUM    8
#else
#define ADS1256_CHANNEL_NUM    4
#endif
 

//*************************** o¨º?¡§¨°?***********************************************/
/*Registers' Address*/
#define REG_STATUS  0   
#define REG_MUX     1	
#define REG_ADCON   2	
#define REG_DRATE   3
#define REG_IO      4	
#define REG_OFC0    5
#define REG_OFC1    6
#define REG_OPC2    7
#define REG_FSC0    8
#define REG_FSC1    9
#define REG_FSC2    10

/*Operation Command*/
//#define CMD_WAKEUP     0x00
#define CMD_RDATA      0x01
#define CMD_RDATAC     0x03
#define CMD_SDATAC     0x0F																												  
#define CMD_RREG       0x10
#define CMD_WREG       0x50
#define CMD_SELFCAL    0xf0
#define CMD_SELFOCAL   0xf1
#define CMD_SELFGCAL   0xf2
#define CMD_SYSOCAL    0xf3
#define CMD_SYSGCAL    0xf4
#define CMD_SYNC       0xfc
#define CMD_STANDBY    0xfd
#define CMD_RESET      0xfe
#define CMD_WAKEUP     0xFF

#define PGA_1            0x00  //¡À5V
#define PGA_2            0x01  //¡À2.5V
#define PGA_4            0x02  //¡À1.25V
#define PGA_8            0x03  //¡À0.625V
#define PGA_16           0x04  //¡À312.5mV
#define PGA_32           0x05  //¡À156.25mV
#define PGA_64           0x06  //¡À78.125mV

#define POSITIVE_AIN0            (0X00<<4)
#define POSITIVE_AIN1            (0X01<<4)
#define POSITIVE_AIN2            (0X02<<4)
#define POSITIVE_AIN3            (0X03<<4)
#define POSITIVE_AIN4            (0X04<<4)
#define POSITIVE_AIN5            (0X05<<4)
#define POSITIVE_AIN6            (0X06<<4)
#define POSITIVE_AIN7            (0X07<<4)
#define POSITIVE_AINCOM          (0X08<<4)        

#define NEGTIVE_AIN0              0X00
#define NEGTIVE_AIN1              0X01
#define NEGTIVE_AIN2              0X02
#define NEGTIVE_AIN3              0X03
#define NEGTIVE_AIN4              0X04
#define NEGTIVE_AIN5              0X05
#define NEGTIVE_AIN6              0X06
#define NEGTIVE_AIN7              0X07
#define NEGTIVE_AINCOM            0X08

/*For fclkin=7.68MHz, data rate*/
#define DATARATE_30K              0xf0
#define DATARATE_15K              0xe0
#define DATARATE_7_5K             0xd0
#define DATARATE_3_7_5K           0xc0
#define DATARATE_2K               0xb0

/*STATUS REGISTER*/
#define MSB_FRIST                (0x00<<3)
#define LSB_FRIST                (0x01<<3)
#define ACAL_OFF                 (0x00<<2)
#define ACAL_ON                  (0x01<<2)
#define BUFEN_OFF                (0x00<<1)
#define BUFEN_ON                 (0x01<<1)

/*ADCON REGISTER*/
#define CLKOUT_OFF               (0x00<<5)
#define CLKOUT_CLKIN             (0x01<<5)
#define DETECT_OFF               (0x00<<3)
#define DETECT_ON_2UA            (0x02<<3)
 
#pragma pack(1)
typedef struct
{
    uint8_t gain;		/* GAIN  */
    uint8_t sampling_rate;	/* DATA output  speed*/  
    uint8_t input_mode;          /*single-ended input channel:1=enable, 0=disable*/
    uint16_t report_interval_ms; /**/
 
    struct{
        uint8_t ADS1256_SINGLE_CH0:1;
        uint8_t ADS1256_SINGLE_CH1:1;
        uint8_t ADS1256_SINGLE_CH2:1;
        uint8_t ADS1256_SINGLE_CH3:1;
        uint8_t ADS1256_SINGLE_CH4:1;
        uint8_t ADS1256_SINGLE_CH5:1;
        uint8_t ADS1256_SINGLE_CH6:1;
        uint8_t ADS1256_SINGLE_CH7:1;
    }single_input_channel;
 
    struct{
        uint8_t ADS1256_DIFF_CH0:1;
        uint8_t ADS1256_DIFF_CH1:1;
        uint8_t ADS1256_DIFF_CH2:1;
        uint8_t ADS1256_DIFF_CH3:1; 
    }diff_input_channel;
 
}ads125x_conf_t; 
#pragma pack()

  
typedef struct{
    
    uint8_t channel_num;
    int32_t adc_sum[ADS1256_CHANNEL_NUM];
    uint32_t adc_cout[ADS1256_CHANNEL_NUM];
    int32_t adc_result[ADS1256_CHANNEL_NUM];	 /* ADC  Conversion value */ 
    int32_t voltage_uv[ADS1256_CHANNEL_NUM];	 /* channel voltage*/ 
}ads125x_channel_info_t; 

/* USER CODE BEGIN Prototypes */

uint8_t ads1256_init(void);
void ads1256_drdy_isr(void);



/* USER CODE END Prototypes */


#endif 

 