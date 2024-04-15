/*
 * 25AA040A.h
 *
 *  Created on: Apr 11, 2024
 *      Author: Ludovic Provost
 *
 *  https://ww1.microchip.com/downloads/en/DeviceDoc/21827H.pdf
 */

#ifndef INC_25AA040A_H_
#define INC_25AA040A_H_

#include "stm32f3xx_hal.h" /* needed for SPI & GPIO */

/*
 * DEFINES
 */
#define NINE_BIT_ADDR_MASK 0x01FF
#define LOWER_BYTE_ADDR_MASK 0x00FF
#define MSB_ADDR_MASK 0x0100

#define PAGE_SIZE 15 // in bytes. Was previously 16, but tested that it wouldnt write 16...
#define EEPROM_PAGE_SIZE_HEX 0x0010
#define EEPROM_MAX_ADDR 0x01FF

/*
 * GPIO CS DEFINE
 */
#define GPIOx GPIOA
#define GPIO_Pin GPIO_PIN_4

/*
 * STATUS REGISTER MASKS
 */
#define SR_WIP_MASK 0x01
#define SR_WEL_MASK 0x02
#define SR_BP_MASK 0x0C

#define SR_WIP_NEG_MASK 0xFE
#define SR_WEL_NEG_MASK 0xFD
#define SR_BP_NEG_MASK 0xF3

typedef uint16_t eeprom_addr_t; //custom data type for 9-bit addr

/* enum for array protection levels (p.11) */
typedef enum ARRAY_PROT {
	NONE = 0b00000000,
	UPPER_QUARTER = 0b00000100,
	UPPER_HALF = 0b00001000,
	ALL = 0b00001100
} ARRAY_PROT;


typedef struct {
	/* SPI handle */
	SPI_HandleTypeDef *hspi;
	uint8_t SR;
} EEPROM;


/*
 * INITIALISATION
 */
uint8_t EEPROM_Initialise(EEPROM *dev, SPI_HandleTypeDef *hspi);

/*
 * INSTRUCTION CALLS
 */
HAL_StatusTypeDef EEPROM_ReadAddr(EEPROM *dev, eeprom_addr_t addr, uint8_t *spi_buf, uint16_t size);
HAL_StatusTypeDef EEPROM_WriteAddr(EEPROM *dev, eeprom_addr_t addr, uint8_t *spi_buf, uint16_t size);
HAL_StatusTypeDef EEPROM_WritePage(EEPROM *dev, eeprom_addr_t addr, uint8_t *spi_buf);
HAL_StatusTypeDef EEPROM_ResetWriteEnableLatch(EEPROM *dev);
HAL_StatusTypeDef EEPROM_SetWriteEnableLatch(EEPROM *dev);

HAL_StatusTypeDef EEPROM_ReadSR(EEPROM *dev, uint8_t *spi_buf);
HAL_StatusTypeDef EEPROM_WriteSR(EEPROM *dev, uint8_t *spi_buf);

/*
 * STATUS BIT ACQUISITION
 */
HAL_StatusTypeDef EEPROM_UpdateSR(EEPROM *dev);
ARRAY_PROT EEPROM_GetArrayProtection(EEPROM *dev);
uint8_t EEPROM_GetWEL(EEPROM *dev);
uint8_t EEPROM_GetWIP(EEPROM *dev);

/*
 * SET STATUS BIT
 */
HAL_StatusTypeDef EEPROM_SetBlockProtection(EEPROM *dev, ARRAY_PROT lvl);

#endif /* INC_25AA040A_H_ */
