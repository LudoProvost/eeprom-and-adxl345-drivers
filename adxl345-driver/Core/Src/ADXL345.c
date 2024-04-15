/*
 * ADXL345.c
 *
 *  Created on: Apr 13, 2024
 *      Author: Ludovic Provost
 */

#include "ADXL345.h"
#include "stm32f3xx_hal.h"

/*
 * INITIALISATION
 */
uint8_t ADXL345_Initialise(ADXL345 *dev, SPI_HandleTypeDef *hspi) {

	/* Set struct parameters */
	dev->hspi = hspi;

	dev->acc[0] = 0;
	dev->acc[1] = 0;
	dev->acc[2] = 0;

	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/*
	 * Verify device ID
	 * Returns error code 254: dev_id not matching.
	 */
	uint8_t regData;

	status = ADXL345_ReadRegister(dev, ADXL345_REG_DEVID, &regData);
	errNum += (status != HAL_OK);

	if (regData != ADXL345_DEVICE_ID) {
		return 254;
	}

	/* Call interrupt init function */
	status = ADXL345_Interrupt_Init(dev);
	errNum += (status != HAL_OK);

	/* Put sensor into measurement mode (p.26) */
	regData = 0x08;

	status = ADXL345_WriteRegister(dev, ADXL345_REG_POWER_CTL, &regData);
	errNum += (status != HAL_OK);

	/* Put sensor in full resolution mode & range +/-16g (p.27) & right-justified w/ sign extension*/
	regData = 0x0F; //0x0B for right justified

	status = ADXL345_WriteRegister(dev, ADXL345_REG_DATA_FORMAT, &regData);
	errNum += (status != HAL_OK);

	return errNum;
}

HAL_StatusTypeDef ADXL345_Interrupt_Init(ADXL345 *dev) {

	HAL_StatusTypeDef status;

	/* Functions and interrupt mapping done before enabling interrupts (p.19) */
	uint8_t int_map_data = 0b01111111; // Sending DATA_READY int to INT1 pin (p.27)
	status = ADXL345_WriteRegister(dev, ADXL345_REG_INT_MAP, &int_map_data);

	/* Early error check */
	if (status != HAL_OK) {
		return status;
	}

	/* Enabling interrupts */
	uint8_t int_enable_data = 0b10000000; // DATA_READY interrupt enabled (p.26)
	status = ADXL345_WriteRegister(dev, ADXL345_REG_INT_ENABLE, &int_enable_data);

	return status;
}


/*
 * DATA ACQUISITION
 */
HAL_StatusTypeDef ADXL345_ReadAcceleration(ADXL345 *dev) {

	HAL_StatusTypeDef status;

	/* Raw values from accelerometer */
	uint8_t reg_data[6];

	status = ADXL345_ReadRegisters(dev, ADXL345_REG_DATAX0, (uint8_t *) reg_data, 6);

	/* Combine registers to give raw (unsigned) readings */
	uint16_t raw_data[3];

	raw_data[0] = (((uint16_t) reg_data[1] << 8) | reg_data[0]);	// X-axis
	raw_data[1] = (((uint16_t) reg_data[3] << 8) | reg_data[2]);	// Y-axis
	raw_data[2] = (((uint16_t) reg_data[5] << 8) | reg_data[4]);	// Z-axis

	/* Convert to signed integers using 2's complement and shifts result 3 bits to the right since LSB is @ D3 */
	int16_t signed_data[3];

	signed_data[0] = ((int16_t) raw_data[0]) >> 3;
	signed_data[1] = ((int16_t) raw_data[1]) >> 3;
	signed_data[2] = ((int16_t) raw_data[2]) >> 3;

	/* Conversion to mps^2 (given range of +- 16g) */
	dev->acc[0] = 9.81f * 0.0039f * signed_data[0];
	dev->acc[1] = 9.81f * 0.0039f * signed_data[1];
	dev->acc[2] = 9.81f * 0.0039f * signed_data[2];

	return status;
}

HAL_StatusTypeDef ADXL345_ReadRawAcceleration(ADXL345 *dev, uint16_t *data_buf) {
	HAL_StatusTypeDef status;

	/* Raw values from accelerometer */
	uint8_t reg_data[6];

	status = ADXL345_ReadRegisters(dev, ADXL345_REG_DATAX0, (uint8_t *) reg_data, 6);

	data_buf[0] = (((uint16_t) reg_data[1] << 8) | reg_data[0]);	// X-axis
	data_buf[1] = (((uint16_t) reg_data[3] << 8) | reg_data[2]);	// Y-axis
	data_buf[2] = (((uint16_t) reg_data[5] << 8) | reg_data[4]);	// Z-axis

	return status;
}


/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef ADXL345_WriteRegister(ADXL345 *dev, uint8_t reg, uint8_t *data) {
	HAL_StatusTypeDef status;

	/* Create first byte of transfer */
	uint8_t instr_addr_byte = ADXL345_WR_NO_MB_MASK;

	instr_addr_byte |= reg;

	/* Create data transfer (2 bytes) */
	uint8_t spi_buf[WRITE_SINGLE_BYTE_SIZE] = {instr_addr_byte, *data};

	/* SPI START */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // select chip line LOW

	status = HAL_SPI_Transmit(dev->hspi, spi_buf, WRITE_SINGLE_BYTE_SIZE, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // select chip line HIGH
	/* SPI END */

	return status;
}

HAL_StatusTypeDef ADXL345_ReadRegister(ADXL345 *dev, uint8_t reg, uint8_t *data_buf) {

	HAL_StatusTypeDef status;

	/* Create first byte of transfer */
	uint8_t instr_addr_byte = ADXL345_RD_NO_MB_MASK;

	instr_addr_byte |= reg;

	/* SPI START */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // select chip line LOW

	status = HAL_SPI_Transmit(dev->hspi, &instr_addr_byte, 1, HAL_MAX_DELAY);

	/* Early error check */
	if (status != HAL_OK) {
		return status;
	}

	status = HAL_SPI_Receive(dev->hspi, data_buf, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // select chip line HIGH
	/* SPI END */

	return status;
}

HAL_StatusTypeDef ADXL345_ReadRegisters(ADXL345 *dev, uint8_t reg, uint8_t *data_buf, uint16_t size) {

	HAL_StatusTypeDef status;

	/* Create first byte of transfer */
	uint8_t instr_addr_byte = ADXL345_RD_MB_MASK;

	instr_addr_byte |= reg;

	/* SPI START */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // select chip line LOW

	status = HAL_SPI_Transmit(dev->hspi, &instr_addr_byte, 1, HAL_MAX_DELAY);

	/* Early error check */
	if (status != HAL_OK) {
		return status;
	}

	status = HAL_SPI_Receive(dev->hspi, data_buf, size, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); // select chip line HIGH
	/* SPI END */

	return status;
}

/*
 * GET DATA FUNCTIONS
 */
float ADXL345_GetX(ADXL345 *dev) {
	return dev->acc[0];
}

float ADXL345_GetY(ADXL345 *dev) {
	return dev->acc[1];
}

float ADXL345_GetZ(ADXL345 *dev) {
	return dev->acc[2];
}
