/*
 * ADXL345.c
 *
 *  Created on: Apr 11, 2024
 *      Author: Ludovic Provost
 */


#include "25AA040A.h"
#include "stm32f3xx_hal.h" /* needed for SPI & GPIO */
#include <stdbool.h>

/*
 * INSTRUCTIONS (p.7)
 * x and A8 bits have been entered as 0.
 */
const uint8_t EEPROM_READ = 0x03;
const uint8_t EEPROM_WRITE = 0x02;
const uint8_t EEPROM_WRDI = 0x04;
const uint8_t EEPROM_WREN = 0x06;
const uint8_t EEPROM_RDSR = 0x05;
const uint8_t EEPROM_WRSR = 0x01;

/*
 * INITIALISATION
 */
uint8_t EEPROM_Initialise(EEPROM *dev, SPI_HandleTypeDef *hspi) {

	/* Set struct parameters */
	dev->hspi = hspi;

	/* Store # of errors*/
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/* Pull CS line HIGH (active low) */
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

	/* Set write enable latch */
	status = EEPROM_SetWriteEnableLatch(dev);
	errNum += (status != HAL_OK);

	/* Set protection level to NONE */
	status = EEPROM_SetBlockProtection(dev, NONE);
	errNum += (status != HAL_OK);

	/* Update dev->SR */
	status = EEPROM_UpdateSR(dev);
	errNum += (status != HAL_OK);


	return errNum;
}

/*
 * INSTRUCTION CALLS (p. 7)
 */

HAL_StatusTypeDef EEPROM_ReadAddr(EEPROM *dev, eeprom_addr_t addr, uint8_t *spi_buf, uint16_t size) {

	HAL_StatusTypeDef status;

	// create lower addr byte
	uint8_t lower_addr_byte = addr & LOWER_BYTE_ADDR_MASK;

	// create instruction + addr MSB byte
	uint8_t instr_byte = EEPROM_READ | (uint8_t) ((addr & MSB_ADDR_MASK) >> 5);

	// create data to be sent (instr + addr)
	uint8_t data[2] = {instr_byte, lower_addr_byte};

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // select chip line LOW
	status = HAL_SPI_Transmit(dev->hspi, data, 2 , HAL_MAX_DELAY); // transmit instruction

	/* Early error check */
	if (status != HAL_OK) {
		return status;
	}

	status = HAL_SPI_Receive(dev->hspi, spi_buf, size, HAL_MAX_DELAY);	// receive SR data

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // select chip line HIGH

	return status;
}

/*
 * make sure to pass sizeof(spi_buf) for size so no problems occur with array
 */
HAL_StatusTypeDef EEPROM_WriteAddr(EEPROM *dev, eeprom_addr_t addr, uint8_t *spi_buf, uint16_t size) {

	HAL_StatusTypeDef status;

	/* size less than page size check */
	int sequence_size;
	if (size > PAGE_SIZE) {
		sequence_size = 2 + PAGE_SIZE;
	} else {
		sequence_size = 2 + size;
	}
	// create lower addr byte
	uint8_t lower_addr_byte = addr & LOWER_BYTE_ADDR_MASK;

	// create instruction + addr MSB byte
	uint8_t instr_byte = EEPROM_WRITE | (uint8_t) ((addr & MSB_ADDR_MASK) >> 5);

	// create data
	uint8_t data[sequence_size]; 	// changes index 0 later

	data[0] = instr_byte;
	data[1] = lower_addr_byte;
	for (int i = 0; i < size; i++) {
		data[2+i] = *(spi_buf+i);
	}

	EEPROM_SetWriteEnableLatch(dev);

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // select chip line LOW
	status = HAL_SPI_Transmit(dev->hspi, data, sequence_size , HAL_MAX_DELAY); // transmit instruction
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // select chip line HIGH

	return status;
}


/*
 * If the address given is less than a page away from the end of the array (0x1FF),
 * it writes enough data to fit in the rest of the page, but does not send more. (p. 8)
 */
HAL_StatusTypeDef EEPROM_WritePage(EEPROM *dev, eeprom_addr_t addr, uint8_t *spi_buf) {

	HAL_StatusTypeDef status;
	int data_size = PAGE_SIZE; // page size by default, will be changed if array end is near

	// addr check
	int bytes_to_array_end = EEPROM_MAX_ADDR - addr;
	if (bytes_to_array_end > EEPROM_PAGE_SIZE_HEX) {
		data_size = bytes_to_array_end;
	}

	int sequence_size = 2 + data_size;

	// create lower addr byte
	uint8_t lower_addr_byte = addr & LOWER_BYTE_ADDR_MASK;

	// create instruction + addr MSB byte
	uint8_t instr_byte = EEPROM_WRITE | (uint8_t) ((addr & MSB_ADDR_MASK) >> 5);

	// create data
	uint8_t data[sequence_size]; 	// changes index 0 later

	data[0] = instr_byte;
	data[1] = lower_addr_byte;
	for (int i = 0; i < data_size; i++) {
		data[2+i] = *(spi_buf+i);
	}

	EEPROM_SetWriteEnableLatch(dev);

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // select chip line LOW
	status = HAL_SPI_Transmit(dev->hspi, data, sequence_size , HAL_MAX_DELAY); // transmit instruction
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // select chip line HIGH

	return status;

	return status;
}

HAL_StatusTypeDef EEPROM_ResetWriteEnableLatch(EEPROM *dev) {

	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // select chip line LOW
	status = HAL_SPI_Transmit(dev->hspi, (uint8_t *) &EEPROM_WRDI, 1, HAL_MAX_DELAY); // transmit instruction
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // select chip line HIGH

	return status;
}

HAL_StatusTypeDef EEPROM_SetWriteEnableLatch(EEPROM *dev) {

	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // select chip line LOW
	status = HAL_SPI_Transmit(dev->hspi, (uint8_t *) &EEPROM_WREN, 1, HAL_MAX_DELAY); // transmit instruction
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // select chip line HIGH

	return status;
}

/*
 * Known issue: HAL_SPI_Receive currently transmits spi_buf when called.
 * Calls HAL_SPI_TransmitReceive() if full-duplex-master enabled. TODO test half-duplex mode.
 * Problematic since EEPROM IC can receive instruction on MOSI while its writing to MISO. could receive wrong instruction.
 */
HAL_StatusTypeDef EEPROM_ReadSR(EEPROM *dev, uint8_t *spi_buf) {

	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // select chip line LOW
	status = HAL_SPI_Transmit(dev->hspi, (uint8_t *) &EEPROM_RDSR, 1, HAL_MAX_DELAY);

	/* Early error check */
	if (status != HAL_OK) {
		return status;
	}

	status = HAL_SPI_Receive(dev->hspi, spi_buf, 1, HAL_MAX_DELAY);	// receive SR data
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // select chip line HIGH

	return status;
}

HAL_StatusTypeDef EEPROM_WriteSR(EEPROM *dev, uint8_t *spi_buf) {

	uint8_t data[2] = {EEPROM_WRSR, *spi_buf};

	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // select chip line LOW
	status = HAL_SPI_Transmit(dev->hspi, (uint8_t *) &data, 2, HAL_MAX_DELAY); // transmit instruction + data
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // select chip line HIGH

	return status;
}

/*
 * STATUS BIT ACQUISITION
 */

HAL_StatusTypeDef EEPROM_UpdateSR(EEPROM *dev) {

	HAL_StatusTypeDef status;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // select chip line LOW
	status = HAL_SPI_Transmit(dev->hspi, (uint8_t *) &EEPROM_RDSR, 1, HAL_MAX_DELAY);

	/* Early error check */
	if (status != HAL_OK) {
		return status;
	}

	status = HAL_SPI_Receive(dev->hspi, &dev->SR, 1, HAL_MAX_DELAY);	// receive SR data
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // select chip line HIGH

	return status;
}

ARRAY_PROT EEPROM_GetArrayProtection(EEPROM *dev) {

	EEPROM_UpdateSR(dev); /* update dev->SR */
	return (dev->SR & SR_BP_MASK);
}

/*
 * returns value of WEL bit. (SR masked and shifted)
 * return:
 * 		0: if WEL bit is 0
 * 		1: if WEL bit is 1
 */
uint8_t EEPROM_GetWEL(EEPROM *dev) {

	EEPROM_UpdateSR(dev); /* update dev->SR */
	return (dev->SR & SR_WEL_MASK)>>1;
}

/*
 * returns value of WIP bit. (SR masked)
 * return:
 * 		0: if WIP bit is 0
 * 		1: if WIP bit is 1
 */
uint8_t EEPROM_GetWIP(EEPROM *dev) {

	EEPROM_UpdateSR(dev); /* update dev->SR */
	return (dev->SR & SR_WIP_MASK);
}

/*
 * SET STATUS BIT
 */

/*
 * see p. 11 of doc for more info on the array protection
 */
HAL_StatusTypeDef EEPROM_SetBlockProtection(EEPROM *dev, ARRAY_PROT lvl) {

	HAL_StatusTypeDef status;

	EEPROM_UpdateSR(dev); /* update dev->SR */

	uint8_t SR_buf = (dev->SR & SR_BP_NEG_MASK) | lvl;
	uint8_t WEL_FLAG = (SR_buf & SR_WEL_MASK);

	status = EEPROM_WriteSR(dev, &SR_buf);

	/* Early error check */
	if (status != HAL_OK) {
		return status;
	}

	/* check WEL status bit. If it was ON, call SetWriteEnableLatch to set it back to 1 after WriteSR */
	if (WEL_FLAG != 0) {
		status = EEPROM_SetWriteEnableLatch(dev);
	}

	return status;
}
