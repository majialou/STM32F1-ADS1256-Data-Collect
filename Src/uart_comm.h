/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_COMM_H
#define __UART_COMM_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Private defines */
#define UART_TRBUF_SIZE   128 


/* USER CODE END Private defines */

 
#pragma pack(1)
typedef struct{
    uint16_t txlen;
    uint8_t  txbuf[UART_TRBUF_SIZE];
    
    uint16_t rxlen;    
    uint8_t  rxbuf[UART_TRBUF_SIZE];
}uart_data_stuc_t;
#pragma pack()

/* USER CODE BEGIN Prototypes */
void uart_init( void );
void uart_idle_rxcb( void );
void uart_send_msg( uint8_t *msg, uint8_t size, uint8_t num ); 

/* USER CODE END Prototypes */

 
#endif