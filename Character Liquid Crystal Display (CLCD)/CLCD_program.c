#include "BIT_MATH.h"
#include "STD_TYPES.h"
#include"DIO_interface.h"
#include "CLCD_register.h"
#include "CLCD_private.h"
#include "CLCD_config.h"
#include"util/delay.h"
#include "CLCD_interface.h"


/*Description:This API for CLCD Intialization */


void CLCD_voidInitalization(void)
{
  _delay_ms(40);
  CLCD_WriteCommand(0b00111000);
  CLCD_WriteCommand(0b00001100);
  _delay_ms(1);
  CLCD_WriteCommand(1);
  _delay_ms(3);

}
/*Description:This API shall display Data on CLCD*/
void CLCD_voidWriteData(u8 Copy_u8Data)
{
  /*Set RS =1*/
   SetPinValue(CLCD_u8_RS_PORT,CLCD_u8_RS_PIN,1);
  /* Set RW =0  */
   SetPinValue(CLCD_u8_RW_PORT,CLCD_u8_RW_PIN,0);
  /*Set DatA  */ 
  SetPinValue(CLCD_u8_D0_PORT,CLCD_u8_D0_PIN,GET_BIT(Copy_u8Data,0));
  SetPinValue(CLCD_u8_D1_PORT,CLCD_u8_D1_PIN,GET_BIT(Copy_u8Data,1));
  SetPinValue(CLCD_u8_D2_PORT,CLCD_u8_D2_PIN,GET_BIT(Copy_u8Data,2));
  SetPinValue(CLCD_u8_D3_PORT,CLCD_u8_D3_PIN,GET_BIT(Copy_u8Data,3));
  SetPinValue(CLCD_u8_D4_PORT,CLCD_u8_D4_PIN,GET_BIT(Copy_u8Data,4));
  SetPinValue(CLCD_u8_D5_PORT,CLCD_u8_D5_PIN,GET_BIT(Copy_u8Data,5));
  SetPinValue(CLCD_u8_D6_PORT,CLCD_u8_D6_PIN,GET_BIT(Copy_u8Data,6));
  SetPinValue(CLCD_u8_D7_PORT,CLCD_u8_D7_PIN,GET_BIT(Copy_u8Data,7));
  /* Enable Pulses  */
  SetPinValue(CLCD_u8_E_PORT,CLCD_u8_E_PIN,1);
  _delay_ms(5);
  SetPinValue(CLCD_u8_E_PORT,CLCD_u8_E_PIN,0);
  _delay_ms(1);
}
static void CLCD_voidWriteHalfData(u8 Copy_u8Data)
{
 /*Set RS =1*/
   SetPinValue(CLCD_u8_RS_PORT,CLCD_u8_RS_PIN,1);
  /* Set RW =0  */
   SetPinValue(CLCD_u8_RW_PORT,CLCD_u8_RW_PIN,0);
  /*Set DatA  */ 
  SetPinValue(CLCD_u8_D0_PORT,CLCD_u8_D4_PIN,GET_BIT(Copy_u8Data,0));
  SetPinValue(CLCD_u8_D1_PORT,CLCD_u8_D5_PIN,GET_BIT(Copy_u8Data,1));
  SetPinValue(CLCD_u8_D2_PORT,CLCD_u8_D6_PIN,GET_BIT(Copy_u8Data,2));
  SetPinValue(CLCD_u8_D3_PORT,CLCD_u8_D7_PIN,GET_BIT(Copy_u8Data,3));
  /* Enable Pulses  */
  SetPinValue(CLCD_u8_E_PORT,CLCD_u8_E_PIN,1);
  _delay_ms(5);
  SetPinValue(CLCD_u8_E_PORT,CLCD_u8_E_PIN,0);
  _delay_ms(1); 
}
/*Description:This API shall excute a command on CLCD*/
void CLCD_WriteCommand(u8 Copy_u8Cm)
{
  /*Set RS =0*/
   SetPinValue(CLCD_u8_RS_PORT,CLCD_u8_RS_PIN,0);
  /* Set RW =0  */
   SetPinValue(CLCD_u8_RW_PORT,CLCD_u8_RW_PIN,0);
  /*Set DatA  */ 
  SetPinValue(CLCD_u8_D0_PORT,CLCD_u8_D0_PIN,GET_BIT(Copy_u8Cm,0));
  SetPinValue(CLCD_u8_D1_PORT,CLCD_u8_D1_PIN,GET_BIT(Copy_u8Cm,1));
  SetPinValue(CLCD_u8_D2_PORT,CLCD_u8_D2_PIN,GET_BIT(Copy_u8Cm,2));
  SetPinValue(CLCD_u8_D3_PORT,CLCD_u8_D3_PIN,GET_BIT(Copy_u8Cm,3));
  SetPinValue(CLCD_u8_D4_PORT,CLCD_u8_D4_PIN,GET_BIT(Copy_u8Cm,4));
  SetPinValue(CLCD_u8_D5_PORT,CLCD_u8_D5_PIN,GET_BIT(Copy_u8Cm,5));
  SetPinValue(CLCD_u8_D6_PORT,CLCD_u8_D6_PIN,GET_BIT(Copy_u8Cm,6));
  SetPinValue(CLCD_u8_D7_PORT,CLCD_u8_D7_PIN,GET_BIT(Copy_u8Cm,7));
  /* Enable Pulses  */
  SetPinValue(CLCD_u8_E_PORT,CLCD_u8_E_PIN,1);
  _delay_ms(1);
  SetPinValue(CLCD_u8_E_PORT,CLCD_u8_E_PIN,0);
  _delay_ms(1);
  
  
  
}
static void CLCD_voidWriteHalfCommand(u8 Copy_uCm)
{
 /*Set RS =1*/
   SetPinValue(CLCD_u8_RS_PORT,CLCD_u8_RS_PIN,0);
  /* Set RW =0  */
   SetPinValue(CLCD_u8_RW_PORT,CLCD_u8_RW_PIN,0);
  /*Set DatA  */ 
  SetPinValue(CLCD_u8_D0_PORT,CLCD_u8_D4_PIN,GET_BIT(Copy_u8Data,0));
  SetPinValue(CLCD_u8_D1_PORT,CLCD_u8_D5_PIN,GET_BIT(Copy_u8Data,1));
  SetPinValue(CLCD_u8_D2_PORT,CLCD_u8_D6_PIN,GET_BIT(Copy_u8Data,2));
  SetPinValue(CLCD_u8_D3_PORT,CLCD_u8_D7_PIN,GET_BIT(Copy_u8Data,3));
  /* Enable Pulses  */
  SetPinValue(CLCD_u8_E_PORT,CLCD_u8_E_PIN,1);
  _delay_ms(5);
  SetPinValue(CLCD_u8_E_PORT,CLCD_u8_E_PIN,0);
  _delay_ms(1); 
}

void CLCD_voidGoToLocation(u8 Copy_LineNb, u8 Copy_Position)
{
	if(Copy_LineNb == CLCD_LINE_1)
	{
		CLCD_WriteCommand(CLCD_LINE_1_CMD + Copy_Position);
	}
	else if(Copy_LineNb == CLCD_LINE_2)
	{
		CLCD_WriteCommand(CLCD_LINE_2_CMD + Copy_Position);
	}
	else
	{}
}

void CLCD_voidWriteString(u8 * Copy_pu8String)
{
	u8 Local_u8Counter=0;
	while(Copy_pu8String[Local_u8Counter] != '\0')
	{
		CLCD_voidWriteData(Copy_pu8String[Local_u8Counter]);
		Local_u8Counter++;
	}
}
