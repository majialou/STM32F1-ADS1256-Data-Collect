# ADS1256数据采集代码

 
The project USES STM32F103 and ADS1256 to build a data acquisition system.
Then the collect data can be display on the ANO_TC匿名上位机V65.
The usr guider of the software can be found in:
https://blog.csdn.net/wangjt1988/article/details/83684188
 
1）MCU: STM32F103C8T6
2) IDE: IAR 7.8.4
3) STM32 CubeMx: 4.27
4) BaudRate:512000

You can modify the sampling rate and channel number to suit your needs in ads1256.c.

ads125x_conf_t ads125x_conf={
    .gain = PGA_1,
    .sampling_rate = DATARATE_30K, 
    .input_mode = ADC1256_INPUT_MODE,
    .report_interval_ms = 5, //uint:ms
    
    /*single-ended input channel:1=enable, 0=disable*/
    .single_input_channel.ADS1256_SINGLE_CH0 = 1,
    .single_input_channel.ADS1256_SINGLE_CH1 = 1,
    .single_input_channel.ADS1256_SINGLE_CH2 = 1,
    .single_input_channel.ADS1256_SINGLE_CH3 = 1,
    .single_input_channel.ADS1256_SINGLE_CH4 = 0,
    .single_input_channel.ADS1256_SINGLE_CH5 = 0,
    .single_input_channel.ADS1256_SINGLE_CH6 = 0,
    .single_input_channel.ADS1256_SINGLE_CH7 = 0,
    
    /*differential input channel:1=enable, 0=disable*/
    .diff_input_channel.ADS1256_DIFF_CH0 = 1, /*AINp=AIN0, AINn=AIN1*/
    .diff_input_channel.ADS1256_DIFF_CH1 = 0, /*AINp=AIN2, AINn=AIN3*/
    .diff_input_channel.ADS1256_DIFF_CH2 = 0, /*AINp=AIN4, AINn=AIN5*/
    .diff_input_channel.ADS1256_DIFF_CH3 = 0, /*AINp=AIN6, AINn=AIN7*/
};
What is more, you can configure aboe parameters through the serial port as you want and save them in flash of the mcu.
The communication protocol is defined in main.c 

/**
* @brief  Parse the packets from the serial port to confirm  
* @param  htim : TIM handle
* @retval None
* reference to data struct "ads125x_conf_t"
*  write configuration to flash
* | 1byte | 1byte |  1byte  | 1byte  |   1byte   |  1byte  |   2byte  |   1byte    |  1byte    | 1byte | 
* | 0xFE  | 0x5A  | LEN=0x7 | GAIN   | DATA RATE | IN MODE | INTERVAL | Single AIN | Diff AIN  | 0x5C  |
*  read configuration from flash
* | 1byte | 1byte |  1byte  |  1byte | 
* | 0xFE  | 0xA5  | LEN=0x0 |  0x5C  |
*  single channel AIN0 
*/

static void uart_packet_parse( uart_data_stuc_t *msg )
{  
    if( msg->rxbuf[0] == 0xFE &&  msg->rxbuf[3+msg->rxbuf[2]] == 0x5c){
        if( msg->rxbuf[1] == 0x5A && msg->rxbuf[2] == sizeof(ads125x_conf_t)){
            falsh_write_conf(&msg->rxbuf[3],msg->rxbuf[2]); 
        }
        else if(msg->rxbuf[1] == 0xA5 &&  msg->rxbuf[2] == 0 ){
            
        }
    }  
}
