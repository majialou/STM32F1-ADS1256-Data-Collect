/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STORAGE_H
#define __STORAGE_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define FLASH_USER_START_ADDR    ((uint32_t)0x0800F800) /* Base @ of   62  Kbytes */    
#define FLASH_USER_PAGE_NUMBER    31   /* End @ of user Flash area */

void falsh_write_conf(uint8_t *conf, uint16_t len);
void falsh_read_conf(uint8_t *conf, uint16_t len);

#endif