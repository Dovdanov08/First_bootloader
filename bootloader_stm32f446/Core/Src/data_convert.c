/*
 * data_convert.c
 *
 *  Created on: 2 мая 2023 г.
 *      Author: Intel
 */
#include "data_convert.h"
//#include <st>

extern UART_HandleTypeDef huart2;


//-----private function---------------//

static HAL_StatusTypeDef symtoDigit(uint8_t *ptrSym, uint16_t symLen, uint8_t dig[MAXDIGIT], uint16_t *colum);
//static STATUS_crc crc_cont(uint8_t (*dig)[MAXDIGIT], uint16_t colum,uint8_t *ptrdata, uint16_t *ndata);
//static STATUS_crc crc_cont(uint8_t (*dig)[MAXDIGIT], uint16_t colum);
static DataStatus parsing_data(uint8_t dig[MAXDIGIT], DataToFlash *Flashdata, uint16_t *size, uint32_t *Start_address);

/**	function for convert symbol massive  to two-dimensional integer massive
 *
 */
HAL_StatusTypeDef symtoDigit(uint8_t *sym_massive, uint16_t LEN, uint8_t digit_massive[MAXDIGIT], uint16_t *colum)
{
	uint16_t nsym=0, i=0, k=0;
	unsigned char ch;
	HAL_StatusTypeDef status=HAL_OK;

	while(nsym<=LEN)
	{
		ch=*(sym_massive+nsym);
		if((ch!='\r')&& (ch!=':')&& (ch!='\n'))
		{
			if((ch>='0')&& (ch <='9'))
				digit_massive[k]=ch-'0';
			else
			{
				switch(ch)
				{
					case 'A':
						digit_massive[k]=10;
						break;
					case 'B':
						digit_massive[k]=11;
						break;
					case 'C':
						digit_massive[k]=12;
						break;
					case 'D':
						digit_massive[k]=13;
						break;
					case 'E':
						digit_massive[k]=14;
						break;
					case 'F':
						digit_massive[k]=15;
						break;

				}
			}
			k++;  //count digit in input masssive
		}
		else {
			if(ch =='\n')
			{
				digit_massive[k]=':';
			}
		}
		nsym++;  //count symbols of input massive
	}

	if(nsym<1)					//if input massive not data
	{
		status=HAL_ERROR;
		return status;
	}
	/* Цикл для перехода от 2,0 к 0x20, так как  цифра в массиве хранятся отдельно (di[i][0]=2;dig[i][1]= 0;)
	 * надо чтобы в массиве были в 0х20= 32.
	 * Для этого просто суммируем эдемент массива со следующим. dig[i][k]=(dig[i][2*k]*16)+dig[i][2*k+1];.
	 */
	for(i=0;i<MAXDIGIT;i++)
	{
		if(digit_massive[i]==':')
			break;
		digit_massive[i]=(digit_massive[2*i]<<4)+digit_massive[2*i+1];
	}
	return status;
}


static DataStatus crc_cont(uint8_t dig[MAXDIGIT])
//static STATUS_crc crc_cont(uint8_t (*dig)[MAXDIGIT], uint16_t colum,uint8_t *ptrdata, uint16_t *ndata)     //функция проверки контрольной суммы пакета (просто суммируем всё ба).
{															//просто суммируем все байты строки и отбрасываем все страшие байты и сумма должна быть равна "00"
	int j=0;
	uint8_t sum=0;


//	for(i=0;i<=*colum;i++)									// суммируем
//	{

	for(j=0;j<=(dig[0]+4);j++)
		sum=sum+dig[j];
//	}

	if(sum!=0)
	{
		return error_crc;
	}
	else
		return work_ok;

}
/* Parsing the data into bytes
 * Two-dimensional array ((*digit_massive)[MAXDIGIT]) data parsing of data, record type, address  (struct DataToFlash).
 */
static DataStatus parsing_data(uint8_t digit_massive[MAXDIGIT], DataToFlash *Flashdata, uint16_t *size, uint32_t *start_address)
{
	uint16_t  j=0,k=0;
	uint32_t line_addr=0, offset_addr=0;

	DataStatus status_file=error_data;

	(Flashdata+*size)->command = *(digit_massive+3);
	(Flashdata+*size)->ndata = *digit_massive;

	switch((Flashdata+*size)->command)
	{
		case (START_CMD):
		{
			*start_address=(((*(digit_massive+4))<<8)+*(digit_massive+5))<<16;
			*size=0;
			status_file=start_ok;
			break;
		}
		case (DATA_CMD):
		{
			for(j=0,k=0;j<(((Flashdata+*size)->ndata)+4);j++)
			{

				if((j>3)&&(j<(((Flashdata+*size)->ndata)+4)))
				{
					(Flashdata+*size)->data[k] =* (digit_massive+j);
					k++;
				}
			}
			status_file=work_ok;
			break;
		}
		case(LINE_ADDR):
		{
			//*size=0;
			line_addr=((((uint32_t)(*(digit_massive+4)))<<8)+*(digit_massive+5))<<12;
			if(line_addr!=*start_address)
				status_file=error_addr;
			status_file=line_addr_ok;
			break;
		}
		case(EOF):
		{
			//*size=0;
			status_file=end_ok;
			break;
		}
		default:
		{
			status_file=error_data;
			break;
		}
	}

	if(status_file!=error_data)
	{
		/*Write address in every string Flash data (Start_cmd+)*/
		offset_addr=((uint32_t)*(digit_massive+2)) + ((uint32_t)(*(digit_massive+1))<<8);
		(Flashdata+*size)->address = offset_addr + *start_address;
	}
	else
		(Flashdata+*size)->address=0xFFFF;
	*size+=1;
	return status_file;

}
DataStatus DataConverter(uint8_t *ptrSym, uint16_t symLen,  DataToFlash *Flashdata, uint16_t *size, uint32_t *start_addr)
{
	HAL_StatusTypeDef status=HAL_OK;
	uint8_t digit_massive[MAXDIGIT]={0};
	DataStatus status_file=error_data;

	status = symtoDigit(ptrSym, symLen, digit_massive, size);

	if(status!=HAL_OK)
		return error_data;


	if(crc_cont(digit_massive)!=error_crc)
		status_file = parsing_data(digit_massive, Flashdata, size, start_addr);
	else
		status_file=error_crc;


	return status_file;
}
/*
HAL_StatusTypeDef Data_to_ComPort(DataToFlash *Flashdata, uint16_t *size)
{
	HAL_StatusTypeDef status_transmit=HAL_OK;
	uint8_t error[]="Error_transmit";
	for(int i=0; i<*size;i++)
	{
		status_transmit = HAL_UART_Transmit(&huart2,Flashdata[i].data, (Flashdata[i].ndata+4),100);
		if(status_transmit!=HAL_OK)
		{
			HAL_UART_Transmit(&huart2,error, sizeof(error),100);
			Error_Handler();
		}
	}
	return status_transmit;
}*/

