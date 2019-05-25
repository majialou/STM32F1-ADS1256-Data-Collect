/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h" 
#include "core_cm3.h"
#include "uart_comm.h"
#include "string.h"
#include "stdio.h"


/*
# ano  
ano应用说明  
·移植和修改说明  
··1.添加ano.h和ano.c文件到你的工程  
··2.ano.c 中修改下文中A&B两处  
·查看实例demo  
·ano上位机设置  
··1.高级收码界面使能对用的功能帧  
··2.设置帧格式，根据你定义的数据来设定  
··3.设定数据来源，保存设置  
··4.打开数据波形，选择用户数据，开启对应通道  
作 者：meetwit  
日 期：2019年3月20日  
*/

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

uart_data_stuc_t uart_data_stuc;


/*
*********************************************************************************************************
* name:  
* function:   
* parameter: Vref : 
* The return value:  NULL
*********************************************************************************************************
*/
static void uart_irq_init( void )
{ 
    __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE); 
    HAL_UART_Receive_DMA(&huart1,uart_data_stuc.rxbuf,sizeof(uart_data_stuc.rxbuf));
}

/*
*********************************************************************************************************
* name:  ano_report
* function: uint8_t func 功能字,自定义，0xF1-0xFF; uint8_t*data 要发送的数据部分,uint8_t len 要发送数据的长度，单位：byte 
* parameter: Vref : 将数据增加帧头，功能字以及校验和并且发送到ano上位机
* The return value:  NULL
*********************************************************************************************************
*/
static void ano_report(uint8_t func, uint8_t *data, uint8_t len)
{
    uint8_t i,cnt=0,sum = 0;
    
    uart_data_stuc.txbuf[len+5]=0; //校验数置零
    
    uart_data_stuc.txbuf[cnt++]=0xAA;  //帧头
    uart_data_stuc.txbuf[cnt++]=0x05;  //S_ADDR    
    uart_data_stuc.txbuf[cnt++]=0xAF;  //D_ADDR    
    uart_data_stuc.txbuf[cnt++]=func;   //功能字
    uart_data_stuc.txbuf[cnt++]=len;  //数据长度
   
    for(i=0;i<len;i++)
        uart_data_stuc.txbuf[cnt++]=data[i];             //复制数据
    
    for(i=0;i<cnt;i++)
        sum += uart_data_stuc.txbuf[i];
    
    uart_data_stuc.txbuf[cnt++]=sum;
         
    uart_data_stuc.txlen = cnt;
    
    HAL_UART_Transmit_DMA(&huart1,uart_data_stuc.txbuf,uart_data_stuc.txlen);  //发送数据到串口
    //for(i=0;i<len+5;i++)usart_send_char(send_buf[i]);     //发送数据到串口
}

/*
*********************************************************************************************************
* name:  ano_send
* function: uint8_t func 功能字,自定义，0xF1-0xFF; uint8_t * sp	显示数据的地址,uint8_t sizenum 	数据类型长度;uint8_t len 要发送数据的长度，单位：byte 
* parameter: Vref : 把数组的内容发送到匿名，需要在匿名选好对应长度的接收
* The return value:  NULL
*********************************************************************************************************
*/ 
static void ano_send(uint8_t fun,uint8_t *sp,uint8_t sizenum,uint8_t len)
{
    uint8_t tbuf[80]={0},i,j; 
    uint8_t *p;
    for(i=0;i<len/sizenum;i++){   		 
        p=sp+sizenum*i;      
        for(j=0;j<sizenum;j++){
            tbuf[j+4*i]=(uint8_t)(*(p+3-j)); 		 
        }
    }
    ano_report(fun,tbuf,len);  
}    

/*
*********************************************************************************************************
* name:  
* function:   
* parameter: Vref : 
* The return value:  NULL
*********************************************************************************************************
*/
void uart_send_msg( uint8_t *msg, uint8_t size, uint8_t num )
{  
    ano_send(0xf1,(uint8_t *)msg,size,num);
}

/*
*********************************************************************************************************
  * @brief  Rx Transfer completed callbacks.
* @param  huart: pointer to a UART_HandleTypeDef structure that contains
*                the configuration information for the specified UART module.
* @retval None   
*********************************************************************************************************
*/
void uart_idle_rxcb( void )
{  
    if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)){ 
        __HAL_UART_CLEAR_IDLEFLAG(&huart1); 
        HAL_UART_DMAStop(&huart1); 
        uint16_t temp = hdma_usart1_rx.Instance->CNDTR; 
        uart_data_stuc.rxlen =  sizeof(uart_data_stuc.rxbuf) - temp; 
        
        HAL_UART_Receive_DMA(&huart1,uart_data_stuc.rxbuf,sizeof(uart_data_stuc.rxbuf));
    }  
}

/*
*********************************************************************************************************
* name:  
* function:   
* parameter: Vref : 
* The return value:  NULL
*********************************************************************************************************
*/
void uart_init( void )
{ 
    uart_irq_init();
}