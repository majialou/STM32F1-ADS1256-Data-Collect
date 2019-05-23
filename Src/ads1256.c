/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"
#include "ads1256.h"
#include "core_cm3.h"


#define SPI2_HANDLE_TIMEOUT  10

/* Private variables ---------------------------------------------------------*/
 
ads125x_conf_t ads125x_conf={
    .gain = PGA_1,
    .sampling_rate = DATARATE_30K, 
    
    /*single-ended input channel:1=enable, 0=disable*/
    .single_input_channel.ADS1256_SINGLE_CH0 = 1,
    .single_input_channel.ADS1256_SINGLE_CH1 = 1,
    .single_input_channel.ADS1256_SINGLE_CH2 = 1,
    .single_input_channel.ADS1256_SINGLE_CH3 = 1,
    .single_input_channel.ADS1256_SINGLE_CH4 = 1,
    .single_input_channel.ADS1256_SINGLE_CH5 = 1,
    .single_input_channel.ADS1256_SINGLE_CH6 = 1,
    .single_input_channel.ADS1256_SINGLE_CH7 = 1,
    
    /*differential input channel:1=enable, 0=disable*/
    .diff_input_channel.ADS1256_DIFF_CH0 = 1,
    .diff_input_channel.ADS1256_DIFF_CH1 = 1,
    .diff_input_channel.ADS1256_DIFF_CH2 = 1,
    .diff_input_channel.ADS1256_DIFF_CH3 = 1, 
};

ads125x_channel_info_t ads125x_channel_info; 

extern TIM_HandleTypeDef htim4;



/* Private function prototypes -----------------------------------------------*/

/*
*********************************************************************************************************
*	name:  
*	function:  
*	parameter:  
*	The return value: NULL
*********************************************************************************************************
*/ 
//#pragma optimize=none
void ads1256_delay_us(uint32_t usec) 
{   
    //#define OSC     (72)                                 //¶¨ÒåÎª72M  
    //#define OSC_D   (OSC/7) 
    //    uint32_t i;  
    //    for(i=0; i<OSC_D*usec; i++){ 
    //        ; 
    //        } 
    uint16_t cccnt,pcnt,dcnt;
    pcnt=htim4.Instance->CNT;
    do{
        cccnt = htim4.Instance->CNT;
        dcnt = (cccnt >= pcnt)?(cccnt - pcnt):(0xFFFF - pcnt + cccnt);
    }while(dcnt<usec); 
}

/*
*********************************************************************************************************
*	name: Voltage_Convert
*	function:  Voltage value conversion function
*	parameter: Vref : The reference voltage 3.3V or 5V
*			   voltage : output DAC value 
*	The return value:  NULL
*********************************************************************************************************
*/
static float ads1256_conv2mv(int32_t adc_result)
{ 
    /* Vin = ( (2*Vr) / G ) * ( x / (2^23 -1)) */  
    float vref = 2.5;
    
    float voltage_mv = (float)adc_result *2.0 * vref / 8388607.0 ;
    voltage_mv /= (float)(ads125x_conf.gain+1);
    voltage_mv *= 1000;
   
    return voltage_mv;
}
/*
*********************************************************************************************************
*	name: ads1256_write_reg
*	function: Write the corresponding register
*	parameter: _RegID: register  ID
*			 _RegValue: register Value
*	The return value: NULL
*********************************************************************************************************
*/
static void ads1256_write_reg(uint8_t reg_addr, uint8_t wdata)
{
    uint8_t wrbuf[3]; 
    
    wrbuf[0] = CMD_WREG | reg_addr;	/*Write command register */
    wrbuf[1] = 0; /*Write the register number */
    wrbuf[2] = wdata;
    
    HAL_SPI_Transmit(&hspi2,wrbuf,sizeof(wrbuf),SPI2_HANDLE_TIMEOUT); 	/*send register value */   
} 

/*
*********************************************************************************************************
*	name: ads1256_write_reg
*	function: Write the corresponding register
*	parameter: _RegID: register  ID
*			 _RegValue: register Value
*	The return value: NULL
*********************************************************************************************************
*/
static void ads1256_write_regs(uint8_t reg_addr, uint8_t *msg, uint8_t len)
{
    uint8_t wrbuf[2]; 
    
    wrbuf[0] = CMD_WREG | reg_addr;	/*Write command register */
    wrbuf[1] = len-1; /*Write the register number */
    
    HAL_SPI_Transmit(&hspi2,wrbuf,sizeof(wrbuf),SPI2_HANDLE_TIMEOUT);  
    
    HAL_SPI_Transmit(&hspi2,msg,len,SPI2_HANDLE_TIMEOUT); 	/*send register value */   
} 

/*
*********************************************************************************************************
*	name: ads1256_read_reg
*	function: Read  the corresponding register
*	parameter: _RegID: register  ID
*	The return value: read register value
*********************************************************************************************************
*/
static void ads1256_read_regs(uint8_t reg_addr, uint8_t *msg, uint8_t len)
{ 
    uint8_t wrbuf[2];
    //HAL_GPIO_WritePin(CSN_GPIO_Port,CSN_Pin,GPIO_PIN_RESET);	/* SPI  cs  = 0 */
    
    wrbuf[0] = CMD_RREG | reg_addr;	/*Write command register */
    wrbuf[1] = len-1; /*Write the register number */ 
    HAL_SPI_Transmit(&hspi2,wrbuf,sizeof(wrbuf),SPI2_HANDLE_TIMEOUT); 	/*send register value */  
    
    HAL_GPIO_WritePin(DBG_OUT_GPIO_Port, DBG_OUT_Pin, GPIO_PIN_SET);
    ads1256_delay_us(10);	/*delay time */
    HAL_GPIO_WritePin(DBG_OUT_GPIO_Port, DBG_OUT_Pin, GPIO_PIN_RESET);
    
    HAL_SPI_Receive(&hspi2, msg, len, SPI2_HANDLE_TIMEOUT);  /* Read the register values */  
}

/*
*********************************************************************************************************
*	name: ads1256_write_cmd
*	function: Sending a single byte order
*	parameter: _cmd : command
*	The return value: NULL
*********************************************************************************************************
*/
static void ads1256_write_cmd(uint8_t _cmd)
{ 
    HAL_SPI_Transmit(&hspi2,&_cmd,sizeof(_cmd),SPI2_HANDLE_TIMEOUT); 	/*send comand value */   
} 

/*
*********************************************************************************************************
*	name: ads1256_set_single_channel
*	function: Configuration channel number
*	parameter:  _ch:  channel number  0--7
*	The return value: NULL
*********************************************************************************************************
*/
static void ads1256_set_single_channel(uint8_t _ch)
{ 
    ads1256_write_reg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN connection AINCOM */
}

/*
*********************************************************************************************************
*	name: ads1256_set_diff_channel
*	function: The configuration difference channel
*	parameter:  _ch:  channel number  0--3
*	The return value:  four high status register
*********************************************************************************************************
*/
static void ads1256_set_diff_channel(uint8_t _ch)
{ 
    uint8_t channel = _ch * 2;
    ads1256_write_reg(REG_MUX,(channel << 4) | (channel + 1) ); 
}  

/*
*********************************************************************************************************
*	name: ads1256_read_result
*	function: read ADC value
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static int32_t ads1256_read_result(void)
{
    int32_t read = 0;
    uint8_t buf[3];
    
    ads1256_write_cmd(CMD_RDATA);	/* read ADC command  */
    
    ads1256_delay_us(10);	/*delay time  */
     
    /*Read the sample results 24bit*/ 
    HAL_SPI_Receive(&hspi2, buf, sizeof(buf), SPI2_HANDLE_TIMEOUT);  
      
    read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
    read |= ((uint32_t)buf[1] << 8);  /* Pay attention to It is wrong   read |= (buf[1] << 8) */
    read |= buf[2];
    
    /* Extend a signed number*/
    if (read & 0x800000) {
        read |= 0xFF000000;
    }
    
    return (int32_t)read;
}  

/*
*********************************************************************************************************
*	name: ads1256_drdy_isr
*	function: Collection procedures
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
void ads1256_drdy_isr(void)
{
    uint8_t adc_result_idx = ads125x_channel_info.channel_num;
   
    ads125x_channel_info.channel_num ++;
    if(ads125x_channel_info.channel_num >= ADS1256_CHANNEL_NUM){
        ads125x_channel_info.channel_num = 0;
    }
    
#if ( ADC1256_INPUT_MODE == ADS1256_SIGNGLE_INPUT ) /*  0  Single-ended input  8 channel?? 1 Differential input  4 channe */
    ads1256_set_single_channel(ads125x_channel_info.channel_num);	/*Switch channel mode */
    ads1256_delay_us(2);
    
    ads1256_write_cmd(CMD_SYNC);
    ads1256_delay_us(2);
    
    ads1256_write_cmd(CMD_WAKEUP);
    ads1256_delay_us(10);
//  
    ads125x_channel_info.adc_result[adc_result_idx] = ads1256_read_result();
    ads125x_channel_info.voltage_mv[adc_result_idx] = ads1256_conv2mv( ads125x_channel_info.adc_result[adc_result_idx]);
    
#else
    /*DiffChannal*/  
    ads1256_set_diff_channel(ads125x_channel_info.channel_num);	/* change DiffChannal */
    ads1256_delay_us(2);
    
    ads1256_write_cmd(CMD_SYNC);
    ads1256_delay_us(2);
    
    ads1256_write_cmd(CMD_WAKEUP);
    ads1256_delay_us(10);
    
    ads125x_channel_info.adc_result[adc_result_idx] = ads1256_read_result();	
    ads125x_channel_info.voltage_mv[adc_result_idx] = ads1256_conv2mv( ads125x_channel_info.adc_result[adc_result_idx]);
#endif
} 



/*
*********************************************************************************************************
*	name: ads1256_init(
*	function: The configuration parameters of ADC, gain and data rate
*	parameter: _gain:gain 1-64
*                      _drate:  data  rate
*	The return value: NULL
*********************************************************************************************************
*/

uint8_t ads1256_init(void)
{     
    uint8_t regs_buf[4];
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_NVIC_DisableIRQ(EXTI4_IRQn);
    for(;;){
        HAL_GPIO_WritePin(GPIOA, REST_Pin|PDWN_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOA, REST_Pin|PDWN_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
        
        while(HAL_GPIO_ReadPin(DRYD_GPIO_Port,DRYD_Pin)); 
        ads1256_read_regs(0,regs_buf,sizeof(regs_buf)); 
        
        if( regs_buf[1]==0x01 && regs_buf[2]==0x20 && regs_buf[3]==0xF0 ) 
            break;
        HAL_Delay(100);
    } 
    
    while(HAL_GPIO_ReadPin(DRYD_GPIO_Port,DRYD_Pin)); 
    regs_buf[REG_STATUS]=0xf4;//STATUS REGISTER:Auto-Calibration Enabled,Analog Input Buffer Disabled
    regs_buf[REG_ADCON]=CLKOUT_OFF+DETECT_OFF+ads125x_conf.gain;   //ADCON=00h
    regs_buf[REG_DRATE]=ads125x_conf.sampling_rate;
    ads1256_write_regs(REG_STATUS,regs_buf,sizeof(regs_buf));
    
    while(HAL_GPIO_ReadPin(DRYD_GPIO_Port,DRYD_Pin)); 
    ads1256_read_regs(0,regs_buf,sizeof(regs_buf));
    
    //ads1256_write_cmd(CMD_WAKEUP);
    
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    
    return 1;
} 



