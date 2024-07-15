/*
 * data_convert.h
 *
 *  Created on: 2 мая 2023 г.
 *      Author: Intel
 */

#ifndef DATA_CONVERT_H_
#define DATA_CONVERT_H_

#include "main.h"
#include "stm32f4xx_hal.h"


#endif /* DATA_CONVERT_H_ */

#define MAXDIGIT					100
#define DATASIZE					18
#define Maxcolum					100

#define START_CMD					0x04
#define DATA_CMD					0X00
#define EOF							0x01
#define LINE_ADDR					0x05

typedef enum
{
  crc_ok       	= 0x00,
  error			= 0x01,
} STATUS_crc;

typedef struct
{
	uint8_t ndata;
	uint32_t address;
	uint8_t command;
	uint8_t data[DATASIZE];
} DataToFlash;
/*
 * Enumeration for parsing input data
 */
typedef enum
{
  start_ok		= 0x00,
  work_ok		= 0x01,
  line_addr_ok	= 0x02,
  end_ok 		= 0x03,
  error_addr	= 0x04,
  error_crc		= 0x05,
  error_data	= 0x06
} DataStatus;



//HAL_StatusTypeDef Data_to_ComPort(DataToFlash *Flashdata, uint16_t *size);

DataStatus DataConverter(uint8_t *ptrSym, uint16_t symLen,  DataToFlash *Flashdata, uint16_t *size, uint32_t *start_addr);
