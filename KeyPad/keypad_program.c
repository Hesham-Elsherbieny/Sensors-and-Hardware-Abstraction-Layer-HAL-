#include "std_types.h"
#include "bit_math.h"

#include "DIO_interface.h"

#include "keypad_interface.h"
#include "keypad_config.h"
#include "keypad_private.h"

u8 keypadGetPressedKey (void)
{
	static u8 keypadColPortarr[KEYPAD_COL_NUM]=KEYPAD_COL_PORTS;
	static u8 keypadColPinarr[KEYPAD_COL_NUM]=KEYPAD_COL_PINS;
	
	static u8 keypadRowPortarr[KEYPAD_ROW_NUM]=KEYPAD_ROW_PORTS;
	static u8 keypadRowPinarr[KEYPAD_ROW_NUM]=KEYPAD_ROW_PINS;
	
	static u8 keypadMatrix[KEYPAD_ROW_NUM][KEYPAD_COL_NUM]=KEYPAD_KEYS;
	u8 row,col, pressedKey = KEYPAD_DEFAULT;
	for(row=0;row<KEYPAD_ROW_NUM;row++)
	{
		/*	clear each row once at a time until you find the row with the pressed key	*/
		SetPinValue(keypadRowPortarr[row],keypadRowPinarr[row],0);

		for(col=0;col<KEYPAD_COL_NUM;col++)
		{
			/*	check each col until you find the col with the presed key	*/
			if(!GetPinValue(keypadColPortarr[col],keypadColPinarr[col]))
			{
				pressedKey = keypadMatrix[row][col];
				while(!GetPinValue(keypadColPortarr[col],keypadColPinarr[col]));
			}
		}
		SetPinValue(keypadRowPortarr[row],keypadRowPinarr[row],1);

	}
	return pressedKey;
}
