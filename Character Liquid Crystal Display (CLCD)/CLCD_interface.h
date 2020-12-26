/************************************************************ 
Author :Hesham Elsherbieny
Version:v01
Date:14-1-2020
************************************************************
*/

#ifndef CLCD_INTERFACE_H
#define CLCD_INTERFACE_H

#define CLCD_ROW_NUM	2
#define CLCD_COL_NUM	16

#define CLCD_CLEAR   	1

#define CLCD_LINE_1		1
#define CLCD_LINE_2		2

/*Description:This API for CLCD Intialization */
void CLCD_voidInitalization(void);
/*Description:This API shall display Data on CLCD*/
void CLCD_voidWriteData(u8 Copy_u8Data);
/*Description:This API shall excute a command on CLCD*/
void CLCD_WriteCommand(u8 Copy_u8Cmd);

void CLCD_voidGoToLocation(u8 Copy_LineNb, u8 Copy_Position);

void CLCD_voidWriteString(u8 * Copy_pu8String);

#endif
