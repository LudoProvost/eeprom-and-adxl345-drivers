/*
 * ADXL345.h
 *
 *  Created on: Apr 13, 2024
 *      Author: Ludovic Provost
 *
 *  https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include "stm32f3xx_hal.h" /* Needed for SPI */

/*
 * DEFINES
 */
#define ADXL345_DEVICE_ID		0xE5

#define WRITE_SINGLE_BYTE_SIZE	2

/* Masks for 2 MSB (W/R and MB) (p. 15) */
#define ADXL345_WR_NO_MB_MASK 	0x00
#define ADXL345_RD_NO_MB_MASK 	0x80
#define ADXL345_RD_MB_MASK		0xC0


/*
 * REGISTERS (p.23)
 */
#define ADXL345_REG_DEVID 			0x00
#define ADXL345_REG_THRESH_TAP 		0x1D
#define ADXL345_REG_OFSX			0x1E
#define ADXL345_REG_OFSY			0x1F
#define ADXL345_REG_OFSZ			0x20
#define ADXL345_REG_DUR				0x21
#define ADXL345_REG_LATENT			0x22
#define ADXL345_REG_WINDOW			0x23
#define ADXL345_REG_THRESH_ACT		0x24
#define ADXL345_REG_THRESH_INACT	0x25
#define ADXL345_REG_TIME_INACT		0x26
#define ADXL345_REG_ACT_INACT_CTL	0x27
#define ADXL345_REG_THRESH_FF		0x28
#define ADXL345_REG_TIME_FF			0x29
#define ADXL345_REG_TAP_AXES		0x2A
#define ADXL345_REG_ACT_TAP_STATUS	0x2B
#define ADXL345_REG_BW_RATE			0x2C
#define ADXL345_REG_POWER_CTL		0x2D
#define ADXL345_REG_INT_ENABLE		0x2E
#define ADXL345_REG_INT_MAP			0x2F
#define ADXL345_REG_INT_SOURCE		0x30
#define ADXL345_REG_DATA_FORMAT		0x31
#define ADXL345_REG_DATAX0			0x32
#define ADXL345_REG_DATAX1			0x33
#define ADXL345_REG_DATAY0			0x34
#define ADXL345_REG_DATAY1			0x35
#define ADXL345_REG_DATAZ0			0x36
#define ADXL345_REG_DATAZ1			0x37
#define ADXL345_REG_FIFO_CTL		0x38
#define ADXL345_REG_FIFO_STATUS		0x39

typedef struct {
	SPI_HandleTypeDef *hspi;
	float acc[3];	// {DATAX, DATAY, DATAZ} in m/s^2
} ADXL345;

/*
 * INITIALISATION
 */
uint8_t ADXL345_Initialise(ADXL345 *dev, SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef ADXL345_Interrupt_Init(ADXL345 *dev);

/*
 * DATA ACQUISITION
 */
HAL_StatusTypeDef ADXL345_ReadAcceleration(ADXL345 *dev);
HAL_StatusTypeDef ADXL345_ReadRawAcceleration(ADXL345 *dev, uint16_t *data_buf);

/*
 * LOW-LEVEL FUNCTIONS
 */
HAL_StatusTypeDef ADXL345_WriteRegister(ADXL345 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef ADXL345_ReadRegister(ADXL345 *dev, uint8_t reg, uint8_t *data_buf);
HAL_StatusTypeDef ADXL345_ReadRegisters(ADXL345 *dev, uint8_t reg, uint8_t *data_buf, uint16_t size);

/*
 * GET DATA FUNCTIONS
 */
float ADXL345_GetX(ADXL345 *dev);
float ADXL345_GetY(ADXL345 *dev);
float ADXL345_GetZ(ADXL345 *dev);

#endif /* INC_ADXL345_H_ */
