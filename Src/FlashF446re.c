/*
 * FlashF446re.c
 *
 *  Created on: 4 нояб. 2023 г.
 *      Author: Intel
 */
#include "FlashF446re.h"
#include "data_convert.h"

//DataToFlash *Flashdata;

static void ERASE_flash();																				// ф-ия очистки определнного сектора flash памяти							// ф-ия контроля правильности записи данных во flash
static uint32_t GetSector(uint32_t Address);


//static FLASH_EraseInitTypeDef EraseInitStruct;

HAL_StatusTypeDef flash_control(DataToFlash *FlashData, uint32_t size);
HAL_StatusTypeDef Flash_Program_Byte(DataToFlash *Flashdata, uint16_t *size);

static FLASH_EraseInitTypeDef EraseInitStruct;

HAL_StatusTypeDef flash_control(DataToFlash *Flashdata, uint32_t size){		// контроль правильности записанных данных

	uint32_t i=0;
	uint32_t adr = FLASH_USER_START_ADDR;										// считываем побайтно (так как изначально, uint8_t)
	uint8_t data = *(__IO uint8_t *)adr;
	HAL_StatusTypeDef status;

	while(i<size){
		//data = *(__IO uint32_t *)FLASH_USER_START_ADDR+4;
		if(Flashdata[i].command==0)
		{
			for(int j=0; j<Flashdata[i].ndata; j++)
			{
				if(data==Flashdata[i].data[j])
					status=HAL_OK;
				else
					return HAL_ERROR;
				adr+=1;																//	смещаем адрес на 1 байт
				data = *(__IO uint8_t *)adr;
			}
			/*adr+=1;																//	смещаем адрес на 1 байт
			data = *(__IO uint8_t *)adr;										//  считываем следующий байт*/
		}
		i++;
	}
	return status;
}

HAL_StatusTypeDef Flash_Program_Byte(DataToFlash *Flashdata, uint16_t *size)						//ф-ия для записи информации во flash-память
{
	uint16_t i=0;
	HAL_StatusTypeDef status = HAL_OK;
	HAL_FLASH_Unlock();								//разблокируем память
	ERASE_flash();									//"освобождаем память" начиная с 2-го сектора

	while(i<=*size)
	{
		for(int j=0; j < Flashdata[i].ndata; j++)
		{
			if(Flashdata[i].command==0x00)				//if command '0x00' write massive Flashdata[i].data
			{
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (Flashdata[i].address)++,Flashdata[i].data[j] ) != HAL_OK)
				{
				   /* Error occurred while writing data in Flash memory.*/
					Error_Handler();
				}
			}
		}
		i++;
	}
	status=HAL_FLASH_Lock();							// блокировка flash памяти

	return status;
}
static void ERASE_flash(void)															//ф-ия для удаления
{
	uint32_t FirstSector = 0, NbOfSectors = 0, SECTORError = 0;
	//static uint32_t GetSector(uint32_t Address);

	/* Get the 4st sector to erase */
	FirstSector = GetSector(FLASH_USER_START_ADDR);
	/* Get the number of sector to erase from 4st sector*/
	NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FirstSector;
	EraseInitStruct.NbSectors     = NbOfSectors;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
	    while (1)
	    {
	    	HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
	    }
	}
	HAL_Delay(100);

}


/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
  {
    sector = FLASH_SECTOR_7;
  }
  return sector;
}
