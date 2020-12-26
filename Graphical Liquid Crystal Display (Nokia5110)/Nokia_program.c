/*
 * Nokia_program.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Hesham Elsherbieny
 */

#include "STD_TYPES.h"
#include "BIT_MATH.h"
#include "Delay_interface.h"

#include "DIO_interface.h"
#include "SPI_interface.h"
#include "STK_interface.h"

#include "Nokia_interface.h"
#include "Nokia_private.h"

void Nokia_Init(void)
{
	/* Reset Pulse H 2ms L 2ms H */

	DIO_SetPinVal('A',2,1);
	//STK_BusyDelay(2000);
	delay_ms(2);

	DIO_SetPinVal('A',2,0);
	//STK_BusyDelay(2000);
	delay_ms(2);

	DIO_SetPinVal('A',2,1);

	/* Nokia is 84x48 with y: 0->5, x;0->83 */

	/* Init Commands */

	/* 1- Go for extended mode */
	WriteCmd(0b00100001);

	/*  Temp Coeff Command -> typical curve */
	WriteCmd(0b00000110);

	/*  Bias system  1:48 with 4 steps */
	WriteCmd(0b00010011);

	/*  Set Vop -> Contrast */
	WriteCmd(0xBF);  //BE

	/* 2- Go back for the normal mode */
	WriteCmd(0b00100000);

	/* Display Control Command */
	WriteCmd(0b00001100);

}

void Nokia_Display(u8* DataArr)
{
	u32 i=0;
	/* Set x=0, Y=0 */

	/* Set X=0 */
	WriteCmd(0b01000000);
	/* Set Y=0 */
	WriteCmd(0b10000000);

	for(i=0;i<504;i++)
	{
		WriteData(DataArr[i]);
	}
}

static void WriteData(u8 Data)
{
	/* CLR CE PIN */
	DIO_SetPinVal('A',3,0);

	/* DC = 1 for Data */
	DIO_SetPinVal('A',1,1);

	/* Send Data Over SPI */
	SPI_SendSynch(Data);

	/* Set CE PIN */
	DIO_SetPinVal('A',3,1);

}

static void WriteCmd(u8 Cmd)
{

	/* CLR CE PIN */
	DIO_SetPinVal('A',3,0);

	/* DC = 0 for Command */
	DIO_SetPinVal('A',1,0);

	/* Send Data Over SPI */
	SPI_SendSynch(Cmd);

	/* Set CE PIN */
	DIO_SetPinVal('A',3,1);

}
