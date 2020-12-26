#include "STD_TYPES.h"
#include "BIT_MATH.h"

#include "I2C_interface.h"

#include "EEPROM_interface.h"
#include "EEPROM_private.h"
#include "EEPROM_config.h"

u8 EEPROM_u8WriteDataByte(u16 Copy_u16ByteAddress, u8 Copy_u8Data)
{
	u8 Local_u8ReturnState = STD_TYPES_u8_ERROR_OK;
	/* Start Condition */
	I2C_enuSendStartCondition();

	/* Send Slave Address with W operation */
	I2C_enuSendSlaveAddWithWrite( (EEPROM_FIXED_ADDRESS | (EEPROM_A2_VALUE << 2) | ((Copy_u16ByteAddress >> 8) & 3)) );

	/* Send Byte Address */
	I2C_enuMasterSendDataByte((u8)Copy_u16ByteAddress);

	/* Send Data */
	I2C_enuMasterSendDataByte(Copy_u8Data);


//	/* Send Data */
//	I2C_enuMasterSendDataByte(10);

	/* Stop Condition */
	I2C_voidSendStopCondition();

	return Local_u8ReturnState;
}
u8 EEPROM_u8ReadDataByte(u16 Copy_u16ByteAddress,u8 * Copy_pu8ReturnData)
{
	u8 Local_u8ReturnState = STD_TYPES_u8_ERROR_OK;
	/* Start Condition */
	I2C_enuSendStartCondition();

	/* Send Slave Address With W */
	I2C_enuSendSlaveAddWithWrite( (EEPROM_FIXED_ADDRESS | (EEPROM_A2_VALUE << 2) | ((Copy_u16ByteAddress >> 8) & 3)) );

	/* Send Byte Address */
	I2C_enuMasterSendDataByte((u8)Copy_u16ByteAddress);

	/* Repeated Start Condition */
	I2C_enuSendReStartCondition();

	/* Send Slave Address With R */
	I2C_enuSendSlaveAddWithRead( (EEPROM_FIXED_ADDRESS | (EEPROM_A2_VALUE << 2) | ((Copy_u16ByteAddress >> 8) & 3)) );

	/* Read Data */
	I2C_enuMasterReadDataByte(Copy_pu8ReturnData);

	/* Stop Condition */
	I2C_voidSendStopCondition();

	return Local_u8ReturnState;
}
