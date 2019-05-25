/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h" 
#include "core_cm3.h" 
#include "storage.h"
#include "stdio.h"
#include "string.h"
#include "stm32f1xx_hal_flash_ex.h"

/*
*********************************************************************************************************
*	name:  
*	function:  
*	parameter:  
*	The return value: NULL
*********************************************************************************************************
*/ 

void falsh_write_conf(uint8_t *conf, uint16_t len)
{
    uint8_t loop,i;
    uint32_t last_data=0;
    uint32_t Address = 0, PAGEError = 0;
    FLASH_EraseInitTypeDef EraseInitStruct;
    
    __disable_irq();
    
    HAL_FLASH_Unlock(); 
    
    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages     = FLASH_USER_PAGE_NUMBER;
    
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) { 
        while (1){} 
    }
    
    Address = FLASH_USER_START_ADDR;
    loop = len/sizeof(uint32_t);
    
    for(i=0;i<loop;i++){
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *(uint32_t*)&conf[i]) == HAL_OK)   {
            Address = Address + 4;
        }
        else { 
            while (1){}
        }
    }
    
    loop = len%sizeof(uint32_t);
    if( loop ){ 
        memcpy((uint8_t*)&last_data,&conf[i*sizeof(uint32_t)],loop);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, last_data) != HAL_OK)   { 
            while (1){}
        }
    } 
    
    HAL_FLASH_Lock(); 
    
    __enable_irq(); 
}
/*
*********************************************************************************************************
*	name:  
*	function:  
*	parameter:  
*	The return value: NULL
*********************************************************************************************************
*/ 
void falsh_read_conf(uint8_t *conf, uint16_t len)
{
    uint16_t i;
    uint8_t *pdata = (uint8_t*)FLASH_USER_START_ADDR;
    
    for(i=0;i<len;i++){
        if(pdata[i] != 0xFF){
            break;
        }
    }
    
    if(i >= len){
        falsh_write_conf(conf,len);
    }
    else{ 
        memcpy(conf,(uint8_t*)FLASH_USER_START_ADDR,len);
    }
}