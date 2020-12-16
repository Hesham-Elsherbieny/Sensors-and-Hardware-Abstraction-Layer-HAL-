/********************************************************************************************
 *                                                                                          *
 * File Name   : MCP9600.c                                                                      *
 *                                                                                          *
 * Author      : Hesham M. Elsherbieny                                                      *
 *                                                                                          *
 * contacts:   : h.elsherbieny@gmail.com                                                    *
 *                                                                                          *
 * Date        : Dec 14, 2020                                                               *
 *                                                                                          *
 * Version     : 1.0.1                                                                      *
 *                                                                                          *
 * Description : Specifies the Microchip MPC9600 Thermocouple EMF to Temperature Converter  *
 *               source file that contain basic functional APIs.                            *
 *                                                                                          *
 ********************************************************************************************/

#include "STD_TYPES.h"
#include "BIT_MATH.h"

#include "I2C_interface.h"

#include "MCP9600.h"
#include "MCP9600_cfg.h"

/*********************************** STATIC FUNCTIONS PROTOTYPES ************************************/
static MCP9600State_t MCP9600_I2C_writeRegister(u8 RegisterAddress, u8 RegisterBytesCount, u16 RegisterValue);
static MCP9600State_t MCP9600_I2C_readRegister(u8 RegisterAddress, u8 RegisterBytesCount, u8* RegisterValue );

/********************************* STATIC FUNCTIONS IMPLEMENTATION ************************************/
static MCP9600State_t MCP9600_I2C_writeRegister(u8 RegisterAddress, u8 RegisterBytesCount, u16 RegisterValue)
{
	MCP9600State_t MCP9600_Status = MCP9600_STATE_NOK;
	I2C_Error_States I2C_State = I2C_Ok;
	u8 ByteCounter = 0;

	/* Start Condition */
	I2C_State = I2C_enuSendStartCondition();

	/* Send Slave Address With W */
	I2C_State = I2C_enuSendSlaveAddWithWrite(MCP9600_SENSOR_I2C_ADDRESS);

	/* Set Register Pointer by Sending a Byte of Register Address */
	I2C_State = I2C_enuMasterSendDataByte(RegisterAddress);

	for(ByteCounter=0;ByteCounter<RegisterBytesCount;ByteCounter++)
	{
		/* Send Each Data Byte */
		I2C_State = I2C_enuMasterSendDataByte( (u8) ( RegisterValue>>(8*ByteCounter) ) );
	}

	/* Stop Condition */
	I2C_voidSendStopCondition();

	if (I2C_State == I2C_Ok)
	{
		MCP9600_Status = MCP9600_STATE_OK;
	}

	return MCP9600_Status;
}

static MCP9600State_t MCP9600_I2C_readRegister(u8 RegisterAddress, u8 RegisterBytesCount, u8* RegisterValue )
{
	MCP9600State_t MCP9600_Status = MCP9600_STATE_NOK;
	I2C_Error_States I2C_State = I2C_Ok;
	u8 ByteCounter = 0;

	/************************************* Set Register Pointer *****************************/

	/* Start Condition */
	I2C_State = I2C_enuSendStartCondition();
	/* Send Slave Address With W */
	I2C_State = I2C_enuSendSlaveAddWithWrite(MCP9600_SENSOR_I2C_ADDRESS);
	/* Set Register Pointer by Sending a Byte of Register Address */
	I2C_State = I2C_enuMasterSendDataByte(RegisterAddress);
	/* Stop Condition */
	I2C_voidSendStopCondition();


	/************************************* Read Registers Data *****************************/

	/* Start Condition */
	I2C_State = I2C_enuSendStartCondition();
	/* Send Slave Address With R */
	I2C_State = I2C_enuSendSlaveAddWithRead(MCP9600_SENSOR_I2C_ADDRESS);
	/* Set Register Pointer by Sending a Byte of Register Address */
	I2C_State = I2C_enuMasterSendDataByte(RegisterAddress);

	for(ByteCounter=0;ByteCounter<RegisterBytesCount;ByteCounter++)
	{
		/* Read Each Data Byte */
		I2C_State = I2C_enuMasterReadDataByte( &RegisterValue[ByteCounter] );

		/* Determining whether it's the Pre-last Byte or not. If it is, i'll Set the ACK bit in order to
		 * send NACK after receiving the next LAST Byte */
		if( ByteCounter == (RegisterBytesCount-2) )
		{
			/* Clear Acknowledgement Bit(NACK) to return NACK after receiving the next LAST Byte */
			I2C_voidSetNACK();
		}
	}

	/* Stop Condition */
	I2C_voidSendStopCondition();

	if (I2C_State == I2C_Ok)
	{
		MCP9600_Status = MCP9600_STATE_OK;
	}

	return MCP9600_Status;
}

/***************************************** PUBLIC APIS IMPLEMENTATION ***********************************************/

MCP9600State_t MCP9600_init(void)
{
	MCP9600State_t status = MCP9600_STATE_NOK;

	u8 SensorCfgReg = 0, DeviceCfgReg = 0;

	SensorCfgReg = MCP9600_THERMOCOUPLE_TYPE | MCP9600_FILTER_COEFF;
	DeviceCfgReg = MCP9600_COLD_JUNC_RESOLUTION | MCP9600_ADC_RESOLUTION | MCP9600_MODE;

	I2C_voidMasterInit();

	status = MCP9600_I2C_writeRegister(MCP9600_SENSOR_CONFIG_ADDRESS, MCP9600_SENSOR_CONFIG_BYTES_COUNT, (u16)SensorCfgReg);

#if MCP9600_MODE == MCP9600_DEVICE_CFG_MODE_BURST_MODE
	DeviceCfgReg |= MCP9600_BURST_MODE_TEMP_SAMPLES;
#endif

	status = MCP9600_I2C_writeRegister(MCP9600_DEVICE_CONFIG_ADDRESS, sizeof(DeviceCfgReg), (u16)DeviceCfgReg);

#if MCP9600_ALERT1_OUTPUT_STATE == MCP9600_ALERT_OUTPUT_ENABLE
	u8  Alert1CfgReg = 0;
	Alert1CfgReg = MCP9600_ALERT1_MODE | MCP9600_ALERT1_OUTPUT_LEVEL | MCP9600_ALERT1_TEMP_DIR | MCP9600_ALERT1_TEMP_MONITOR;
	status = MCP9600_I2C_writeRegister(MCP9600_ALERT1_CONFIG_ADDRESS, MCP9600_ALERT1_CONFIG_BYTES_COUNT, (u16)Alert1CfgReg);
#endif

#if MCP9600_ALERT2_OUTPUT_STATE == MCP9600_ALERT_OUTPUT_ENABLE
	u8  Alert2CfgReg = 0;
	Alert2CfgReg = MCP9600_ALERT2_MODE | MCP9600_ALERT2_OUTPUT_LEVEL | MCP9600_ALERT2_TEMP_DIR | MCP9600_ALERT2_TEMP_MONITOR;
	status = MCP9600_I2C_writeRegister(MCP9600_ALERT2_CONFIG_ADDRESS, MCP9600_ALERT2_CONFIG_BYTES_COUNT, (u16)Alert2CfgReg);
#endif

#if MCP9600_ALERT3_OUTPUT_STATE == MCP9600_ALERT_OUTPUT_ENABLE
	u8  Alert3CfgReg = 0;
	Alert3CfgReg = MCP9600_ALERT3_MODE | MCP9600_ALERT3_OUTPUT_LEVEL | MCP9600_ALERT3_TEMP_DIR | MCP9600_ALERT3_TEMP_MONITOR;
	status = MCP9600_I2C_writeRegister(MCP9600_ALERT3_CONFIG_ADDRESS, MCP9600_ALERT3_CONFIG_BYTES_COUNT, (u16)Alert3CfgReg);
#endif

#if MCP9600_ALERT4_OUTPUT_STATE == MCP9600_ALERT_OUTPUT_ENABLE
	u8  Alert4CfgReg = 0;
	Alert4CfgReg = MCP9600_ALERT4_MODE | MCP9600_ALERT4_OUTPUT_LEVEL | MCP9600_ALERT4_TEMP_DIR | MCP9600_ALERT4_TEMP_MONITOR;
	status = MCP9600_I2C_writeRegister(MCP9600_ALERT4_CONFIG_ADDRESS, MCP9600_ALERT4_CONFIG_BYTES_COUNT, (u16)Alert4CfgReg);
#endif


	return status;
}


MCP9600State_t MCP9600_readHotJuncTemp(f32* HotTemp)
{
	MCP9600State_t Status = MCP9600_STATE_NOK;
	u8 DataBytes[MCP9600_HOT_JUNC_TEMP_BYTES_COUNT];

	Status = MCP9600_I2C_readRegister(MCP9600_HOT_JUNC_TEMP_ADDRESS, MCP9600_HOT_JUNC_TEMP_BYTES_COUNT, DataBytes);

	/* Checking the sign of Upper Byte and then convert it to temperature
	 * Temperature Data Conversion is done using the equation provided in DataSheet */
	if((DataBytes[0] & MCP9600_TH_SIGN) == MCP9600_TH_NEGATIVE)
	{
		*HotTemp = ( ( (f32)DataBytes[0]*16 ) + ( (f32)DataBytes[1]/16 ) ) - 4069;
	}
	else if((DataBytes[1] & MCP9600_TH_SIGN) == MCP9600_TH_POSITIVE)
	{
		*HotTemp = ( ( (f32)DataBytes[0]*16 ) + ( (f32)DataBytes[1]/16 ) );
	}

	return Status;
}

MCP9600State_t MCP9600_readColdJuncTemp(f32* ColdTemp)
{
	MCP9600State_t Status = MCP9600_STATE_NOK;
	u8 DataBytes[MCP9600_COLD_JUNC_TEMP_BYTES_COUNT];

	Status = MCP9600_I2C_readRegister(MCP9600_COLD_JUNC_TEMP_ADDRESS, MCP9600_COLD_JUNC_TEMP_BYTES_COUNT, DataBytes);

	/* Checking the sign of Upper Byte and then convert it to temperature
	 * Temperature Data Conversion is done using the equation provided in DataSheet */
	if((DataBytes[0] & MCP9600_TC_SIGN) == MCP9600_TC_NEGATIVE)
	{
		*ColdTemp = ( ( (f32)DataBytes[0]*16 ) + ( (f32)DataBytes[1]/16 ) ) - 4069;
	}
	else if((DataBytes[1] & MCP9600_TC_SIGN) == MCP9600_TC_POSITIVE)
	{
		*ColdTemp = ( ( (f32)DataBytes[0]*16 ) + ( (f32)DataBytes[1]/16 ) );
	}

	return Status;
}

MCP9600State_t MCP9600_readDeltaTemp(f32* DeltaTemp)
{
	MCP9600State_t Status = MCP9600_STATE_NOK;
	u8 DataBytes[MCP9600_DELTA_JUNC_TEMP_BYTES_COUNT];

	Status = MCP9600_I2C_readRegister(MCP9600_DELTA_JUNC_TEMP_ADDRESS, MCP9600_DELTA_JUNC_TEMP_BYTES_COUNT, DataBytes);

	/* Checking the sign of Upper Byte and then convert it to temperature
	 * Temperature Data Conversion is done using the equation provided in DataSheet */
	if((DataBytes[0] & MCP9600_TD_SIGN) == MCP9600_TD_NEGATIVE)
	{
		*DeltaTemp = ( ( (f32)DataBytes[0]*16 ) + ( (f32)DataBytes[1]/16 ) ) - 4069;
	}
	else if((DataBytes[1] & MCP9600_TD_SIGN) == MCP9600_TD_POSITIVE)
	{
		*DeltaTemp = ( ( (f32)DataBytes[0]*16 ) + ( (f32)DataBytes[1]/16 ) );
	}

	return Status;
}

MCP9600State_t MCP9600_readADCVal(f32* ADCVal)
{
	MCP9600State_t Status = MCP9600_STATE_NOK;
	u8 DataBytes[MCP9600_ADC_RAW_DATA_BYTES_COUNT];

	Status = MCP9600_I2C_readRegister(MCP9600_ADC_RAW_DATA_ADDRESS, MCP9600_ADC_RAW_DATA_BYTES_COUNT, DataBytes);

	*ADCVal = (f32) ( (f32)DataBytes[2] ) | ( (f32)DataBytes[1]<<8 ) | ( (f32)DataBytes[0]<<16 );

	return Status;
}

MCP9600State_t MCP9600_getSensorStatus(MCP9600_SensorStatus_t* Status)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 RegVal= 0;

	State = MCP9600_I2C_readRegister(MCP9600_STATUS_ADDRESS, MCP9600_STATUS_BYTES_COUNT, &RegVal);

	if( (RegVal & MCP9600_STATUS_BURST_MODE_CONV_FLAG))
	{
		Status->BurstConvComplete = TRUE;
	}
	else
	{
		Status->BurstConvComplete = FALSE;
	}

	if( (RegVal & MCP9600_STATUS_TEMP_UPDATE_FLAG))
	{
		Status->HotTempUpdate = TRUE;
	}
	else
	{
		Status->HotTempUpdate = FALSE;
	}

	if( (RegVal & MCP9600_STATUS_TEMP_RANGE_DETECT_FLAG))
	{
		Status->InputRangeExceed = TRUE;
	}
	else
	{
		Status->InputRangeExceed = FALSE;
	}

	if( (RegVal & MCP9600_STATUS_ALERT1_STATUS_FLAG))
	{
		Status->TempExceedAlert1 = TRUE;
	}
	else
	{
		Status->TempExceedAlert1 = FALSE;
	}

	if( (RegVal & MCP9600_STATUS_ALERT2_STATUS_FLAG))
	{
		Status->TempExceedAlert2 = TRUE;
	}
	else
	{
		Status->TempExceedAlert2 = FALSE;
	}

	if( (RegVal & MCP9600_STATUS_ALERT3_STATUS_FLAG))
	{
		Status->TempExceedAlert3 = TRUE;
	}
	else
	{
		Status->TempExceedAlert3 = FALSE;
	}

	if( (RegVal & MCP9600_STATUS_ALERT4_STATUS_FLAG))
	{
		Status->TempExceedAlert4 = TRUE;
	}
	else
	{
		Status->TempExceedAlert4 = FALSE;
	}

	return State;
}

MCP9600State_t MCP9600_setSensorConfig(u8 ThermoType, u8 FilterCoeff)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 RegVal = 0;

	RegVal = ( ThermoType | FilterCoeff );

	State = MCP9600_I2C_writeRegister(MCP9600_SENSOR_CONFIG_ADDRESS, MCP9600_SENSOR_CONFIG_BYTES_COUNT, (u16)RegVal);

	return State;
}

MCP9600State_t MCP9600_setDeviceConfig(u8 ColdJuncResol, u8 ADCResol, u8 BurstSampleCount, u8 Mode)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 RegVal = 0;

	RegVal = ( ColdJuncResol | ADCResol | Mode );

	if(Mode == MCP9600_DEVICE_CFG_MODE_BURST_MODE)
	{
		RegVal |= BurstSampleCount;
	}

	State = MCP9600_I2C_writeRegister(MCP9600_DEVICE_CONFIG_ADDRESS, MCP9600_DEVICE_CONFIG_BYTES_COUNT, (u16)RegVal);

	return State;
}

MCP9600State_t MCP9600_setAlertConfig(u8 AlertID, MCP9600_AlertConfig_t AlertConfig)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 RegAddress=0, RegByteCount=0;
	u8 RegVal =0;
	switch(AlertID)
	{
		case ALERT1:
			RegAddress = MCP9600_ALERT1_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT1_CONFIG_BYTES_COUNT;
			break;
		case ALERT2:
			RegAddress = MCP9600_ALERT2_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT2_CONFIG_BYTES_COUNT;
			break;
		case ALERT3:
			RegAddress = MCP9600_ALERT3_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT3_CONFIG_BYTES_COUNT;
			break;
		case ALERT4:
			RegAddress = MCP9600_ALERT4_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT4_CONFIG_BYTES_COUNT;
			break;
		default:
			State = MCP9600_STATE_INVALID_PARAM;
			return State;
	}

	RegVal = AlertConfig.AlertIntClear | AlertConfig.AlertTempType | AlertConfig.AlertTempDir | AlertConfig.AlertState
		   | AlertConfig.AlertMode | AlertConfig.AlertEnable;

	State = MCP9600_I2C_writeRegister(RegAddress, RegByteCount, RegVal);

	return State;
}

MCP9600State_t MCP9600_setAlertLimit(u8 AlertID, f32 AlertLimitTemp)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 RegAddress=0, RegByteCount=0;
	u8 RegVal =0;

	switch(AlertID)
	{
		case ALERT1:
			RegAddress   = MCP9600_ALERT1_LIMIT_ADDRESS;
			RegByteCount = MCP9600_ALERT1_LIMIT_BYTES_COUNT;
			break;
		case ALERT2:
			RegAddress   = MCP9600_ALERT2_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT2_CONFIG_BYTES_COUNT;
			break;
		case ALERT3:
			RegAddress   = MCP9600_ALERT3_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT3_CONFIG_BYTES_COUNT;
			break;
		case ALERT4:
			RegAddress   = MCP9600_ALERT4_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT4_CONFIG_BYTES_COUNT;
			break;
		default:
			State = MCP9600_STATE_INVALID_PARAM;
			return State;
	}

	/* Validate Input Temperature to be within Range */
	if( (AlertLimitTemp > MCP9600_ALERT_MAX_POSITIVE_TEMP_LIMIT) || (AlertLimitTemp < MCP9600_ALERT_MAX_NEGATIVE_TEMP_LIMIT) )
	{
		State = MCP9600_STATE_INVALID_PARAM;
		return State;
	}
	else
	{
		/* Map Temp Value In order to clear first 2 bits as described in datasheet */
		RegVal = AlertLimitTemp * 16;
	}

	State = MCP9600_I2C_writeRegister(RegAddress, RegByteCount, RegVal);

	return State;
}

MCP9600State_t MCP9600_setHystVal(u8 AlertID, u8 HystVal)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 RegAddress=0, RegByteCount=0;
	u8 RegVal =0;

	switch(AlertID)
	{
		case ALERT1:
			RegAddress   = MCP9600_ALERT1_HYSTERESIS_ADDRESS;
			RegByteCount = MCP9600_ALERT1_HYSTERESIS_BYTES_COUNT;
			break;
		case ALERT2:
			RegAddress   = MCP9600_ALERT2_HYSTERESIS_ADDRESS;
			RegByteCount = MCP9600_ALERT2_HYSTERESIS_BYTES_COUNT;
			break;
		case ALERT3:
			RegAddress   = MCP9600_ALERT3_HYSTERESIS_ADDRESS;
			RegByteCount = MCP9600_ALERT3_HYSTERESIS_BYTES_COUNT;
			break;
		case ALERT4:
			RegAddress   = MCP9600_ALERT4_HYSTERESIS_ADDRESS;
			RegByteCount = MCP9600_ALERT4_HYSTERESIS_BYTES_COUNT;
			break;
		default:
			State = MCP9600_STATE_INVALID_PARAM;
			return State;
	}

	/* Validate Input Hysteresis to be within Range */
	if( (HystVal > MCP9600_ALERT_HYST_MAX_TEMP_LIMIT) || (HystVal < MCP9600_ALERT_HYST_MIN_TEMP_LIMIT) )
	{
		State = MCP9600_STATE_INVALID_PARAM;
		return State;
	}
	else
	{
		RegVal = HystVal;
	}

	State = MCP9600_I2C_writeRegister(RegAddress, RegByteCount, RegVal);

	return State;
}

MCP9600State_t MCP9600_getDeviceInfo(MCP9600_DeviceInfo_t* DeviceInfo)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 DataBytes[MCP9600_DEVICE_ID_BYTES_COUNT];

	State = MCP9600_I2C_readRegister(MCP9600_DEVICE_ID_ADDRESS, MCP9600_DEVICE_ID_BYTES_COUNT, DataBytes);

	DeviceInfo->DeviceID      =  DataBytes[0];
	DeviceInfo->MajorRevision = (DataBytes[1] & MCP9600_MAJOR_REVISION_ID) >> 4;
	DeviceInfo->MinorRevision =  DataBytes[1] & MCP9600_MINOR_REVISION_ID;

	return State;
}

MCP9600State_t MCP9600_getSensorConfig(u8* ThermoType, u8* FilterCoeff)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 RegVal = 0;

	State = MCP9600_I2C_readRegister(MCP9600_SENSOR_CONFIG_ADDRESS, MCP9600_SENSOR_CONFIG_BYTES_COUNT, &RegVal);

	*ThermoType  = ( ( RegVal & MCP9600_THERMOCOUPLE_TYPE_MASK ) >> MCP9600_THERMOCOUPLE_TYPE_OFFSET );
	*FilterCoeff =   ( RegVal & MCP9600_FILTER_COEFF_MASK );

	return State;
}

MCP9600State_t MCP9600_getDeviceConfig(u8* ColdJuncResol, u8* ADCResol, u8* BurstSampleCount, u8* Mode)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 RegVal = 0;

	State = MCP9600_I2C_readRegister(MCP9600_DEVICE_CONFIG_ADDRESS, MCP9600_DEVICE_CONFIG_BYTES_COUNT, &RegVal);

	*Mode             = (RegVal & MCP9600_DEVICE_CFG_MODE_MASK);
	*BurstSampleCount = ( (RegVal & MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_MASK) >> MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_OFFSET );
	*ADCResol		  = ( (RegVal & MCP9600_DEVICE_CFG_ADC_RES_MASK) >> MCP9600_DEVICE_CFG_ADC_RES_OFFSET );
	*ColdJuncResol    = ( (RegVal & MCP9600_DEVICE_CFG_COLD_JUNC_RES_MASK) >> MCP9600_DEVICE_CFG_COLD_JUNC_RES_OFFSET );

	return State;
}

MCP9600State_t MCP9600_getAlertConfig(u8 AlertID, MCP9600_AlertConfig_t* AlertConfig)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 RegAddress=0, RegByteCount=0;
	u8 RegVal =0;

	switch(AlertID)
	{
		case ALERT1:
			RegAddress = MCP9600_ALERT1_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT1_CONFIG_BYTES_COUNT;
			break;
		case ALERT2:
			RegAddress = MCP9600_ALERT2_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT2_CONFIG_BYTES_COUNT;
			break;
		case ALERT3:
			RegAddress = MCP9600_ALERT3_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT3_CONFIG_BYTES_COUNT;
			break;
		case ALERT4:
			RegAddress = MCP9600_ALERT4_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT4_CONFIG_BYTES_COUNT;
			break;
		default:
			State = MCP9600_STATE_INVALID_PARAM;
			return State;
	}

	State = MCP9600_I2C_readRegister(RegAddress, RegByteCount, &RegVal);

	AlertConfig->AlertEnable   = RegVal & MCP9600_ALERT_OUTPUT_STATE_MASK;
	AlertConfig->AlertIntClear = (RegVal & MCP9600_ALERT_INTERRUPT_MASK) >> MCP9600_ALERT_INTERRUPT_OFFSET;
	AlertConfig->AlertMode     = (RegVal & MCP9600_ALERT_MODE_MASK) >> MCP9600_ALERT_MODE_OFFSET;
	AlertConfig->AlertState    = (RegVal & MCP9600_ALERT_OUTPUT_ACTIVE_STATE_MASK) >> MCP9600_ALERT_OUTPUT_ACTIVE_STATE_OFFSET;
	AlertConfig->AlertTempDir  = (RegVal & MCP9600_ALERT_LIMIT_MASK) >> MCP9600_ALERT_LIMIT_OFFSET;
	AlertConfig->AlertTempType = (RegVal & MCP9600_ALERT_MONITOR_TEMP_MASK) >> MCP9600_ALERT_MONITOR_TEMP_OFFSET;

	return State;
}

MCP9600State_t MCP9600_getAlertLimit(u8 AlertID, f32* AlertLimitTemp)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 RegAddress=0, RegByteCount=0;
	u8 RegVal =0;

	switch(AlertID)
	{
		case ALERT1:
			RegAddress   = MCP9600_ALERT1_LIMIT_ADDRESS;
			RegByteCount = MCP9600_ALERT1_LIMIT_BYTES_COUNT;
			break;
		case ALERT2:
			RegAddress   = MCP9600_ALERT2_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT2_CONFIG_BYTES_COUNT;
			break;
		case ALERT3:
			RegAddress   = MCP9600_ALERT3_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT3_CONFIG_BYTES_COUNT;
			break;
		case ALERT4:
			RegAddress   = MCP9600_ALERT4_CONFIG_ADDRESS;
			RegByteCount = MCP9600_ALERT4_CONFIG_BYTES_COUNT;
			break;
		default:
			State = MCP9600_STATE_INVALID_PARAM;
			return State;
	}

	State = MCP9600_I2C_readRegister(RegAddress, RegByteCount, &RegVal);

	/* Map Register Value In order to return the original Temperature Without the Cleared First 2 Bits */
	AlertLimitTemp = RegVal / 16;

	return State;
}

MCP9600State_t MCP9600_getHystVal(u8 AlertID, u8* HystVal)
{
	MCP9600State_t State = MCP9600_STATE_NOK;
	u8 RegAddress=0, RegByteCount=0;
	u8 RegVal =0;

	switch(AlertID)
	{
		case ALERT1:
			RegAddress   = MCP9600_ALERT1_HYSTERESIS_ADDRESS;
			RegByteCount = MCP9600_ALERT1_HYSTERESIS_BYTES_COUNT;
			break;
		case ALERT2:
			RegAddress   = MCP9600_ALERT2_HYSTERESIS_ADDRESS;
			RegByteCount = MCP9600_ALERT2_HYSTERESIS_BYTES_COUNT;
			break;
		case ALERT3:
			RegAddress   = MCP9600_ALERT3_HYSTERESIS_ADDRESS;
			RegByteCount = MCP9600_ALERT3_HYSTERESIS_BYTES_COUNT;
			break;
		case ALERT4:
			RegAddress   = MCP9600_ALERT4_HYSTERESIS_ADDRESS;
			RegByteCount = MCP9600_ALERT4_HYSTERESIS_BYTES_COUNT;
			break;
		default:
			State = MCP9600_STATE_INVALID_PARAM;
			return State;
	}

	State = MCP9600_I2C_readRegister(RegAddress, RegByteCount, &RegVal);

	*HystVal = RegVal;

	return State;
}
