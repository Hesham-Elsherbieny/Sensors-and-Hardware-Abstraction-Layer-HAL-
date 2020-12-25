/********************************************************************************************
 *                                                                                          *
 * File Name   : MCP9600.h                                                                  *
 *                                                                                          *
 * Author      : Hesham M. Elsherbieny                                                      *
 *                                                                                          *
 * contacts:   : h.elsherbieny@gmail.com                                                    *
 *                                                                                          *
 * Date        : Dec 14, 2020                                                               *
 *                                                                                          *
 * Version     : 1.0.2                                                                      *
 *                                                                                          *
 * Description : Specifies the Microchip MPC9600 Thermocouple EMF to Temperature Converter  *
 *               header file that contains needed interfaces and configuration masks.       *
 *                                                                                          *
 ********************************************************************************************/

#ifndef MCP9600_H_
#define MCP9600_H_
/************************************** TYPE DEFINITIONS ***************************************************************/

/* Enum type definition of All MCP9600 Possible States */
typedef enum
{
	MCP9600_STATE_NOK,
	MCP9600_STATE_OK,
	MCP9600_STATE_BUSY,
	MCP9600_STATE_INVALID_PARAM
}MCP9600State_t;

/* Data Structure Object that holds all internal Sensor Status Associated with Flag Bits */
typedef struct
{
	bool BurstConvComplete;
	bool HotTempUpdate;
	bool InputRangeExceed;
	bool TempExceedAlert1;
	bool TempExceedAlert2;
	bool TempExceedAlert3;
	bool TempExceedAlert4;
}MCP9600_SensorStatus_t;

/* Data Structure Object that holds all Alert Configuration Register Parametres  */
typedef struct
{
	bool AlertEnable;
	bool AlertIntClear;
	u8   AlertMode;
	u8	 AlertState;
	u8   AlertTempDir;
	u8   AlertTempType;
}MCP9600_AlertConfig_t;

/* Data Structure Object that holds all module informations like Device ID and Revision Number */
typedef struct
{
	u8 DeviceID;
	u8 MajorRevision;
	u8 MinorRevision;
}MCP9600_DeviceInfo_t;

/************************************** BIT MASKS ***************************************************************/

/*
 * MCP9600 Temperature Sensor I2C Fixed Part Of Address
 */
#define MCP9600_I2C_FIXED_ADDRESS_PART	0X60

/*
 * MCP9600 Temperature Sensor I2C Address Levels Which specified by A2, A1, A0 Hardware Address Bits
 * Levels 1,2,3,4,5,6 Can be Obtained by connecting ADDR Pin to Voltage Divider Network with
 * Resistance values as described in DataSheet
 */
#define MCP9600_I2C_ADDRESS_LEVEL_0		0X00  /* ADDR Pin Connected to Ground */
#define MCP9600_I2C_ADDRESS_LEVEL_1		0X01
#define MCP9600_I2C_ADDRESS_LEVEL_2		0X02
#define MCP9600_I2C_ADDRESS_LEVEL_3		0X03
#define MCP9600_I2C_ADDRESS_LEVEL_4		0X04
#define MCP9600_I2C_ADDRESS_LEVEL_5		0X05
#define MCP9600_I2C_ADDRESS_LEVEL_6		0X06
#define MCP9600_I2C_ADDRESS_LEVEL_7		0X07  /* ADDR Pin Connected to VDD */

/*
 * MCP9600 Thermocouple Register Pointer Bits which represent
 * all internal Registers Addresses
 */
#define MCP9600_HOT_JUNC_TEMP_ADDRESS			0x00 /* Thermocouple Hot-Junction register, TH */
#define MCP9600_DELTA_JUNC_TEMP_ADDRESS		    0x01 /* Junctions Temperature Delta register, TD */
#define MCP9600_COLD_JUNC_TEMP_ADDRESS			0x02 /* Cold-Junction Temperature register, TC */
#define MCP9600_ADC_RAW_DATA_ADDRESS		    0x03 /* Raw ADC Data register */
#define MCP9600_STATUS_ADDRESS                  0x04 /* STATUS register */
#define MCP9600_SENSOR_CONFIG_ADDRESS           0x05 /* Thermocouple Sensor Configuration register */
#define MCP9600_DEVICE_CONFIG_ADDRESS			0x06 /* Device Configuration register */
#define MCP9600_ALERT1_CONFIG_ADDRESS			0x08 /* Alert 1 Configuration register */
#define MCP9600_ALERT2_CONFIG_ADDRESS			0x09 /* Alert 2 Configuration register */
#define MCP9600_ALERT3_CONFIG_ADDRESS			0x0A /* Alert 3 Configuration register */
#define MCP9600_ALERT4_CONFIG_ADDRESS			0x0B /* Alert 4 Configuration register */
#define MCP9600_ALERT1_HYSTERESIS_ADDRESS		0x0C /* Alert 1 Hysteresis register, THYST1 */
#define MCP9600_ALERT2_HYSTERESIS_ADDRESS		0x0D /* Alert 2 Hysteresis register, THYST2 */
#define MCP9600_ALERT3_HYSTERESIS_ADDRESS		0x0E /* Alert 3 Hysteresis register, THYST3 */
#define MCP9600_ALERT4_HYSTERESIS_ADDRESS		0x0F /* Alert 4 Hysteresis register, THYST4 */
#define MCP9600_ALERT1_LIMIT_ADDRESS			0x10 /* Temperature Alert 1 Limit register, TALERT1 */
#define MCP9600_ALERT2_LIMIT_ADDRESS			0x11 /* Temperature Alert 2 Limit register, TALERT2 */
#define MCP9600_ALERT3_LIMIT_ADDRESS			0x12 /* Temperature Alert 3 Limit register, TALERT3 */
#define MCP9600_ALERT4_LIMIT_ADDRESS			0x13 /* Temperature Alert 4 Limit register, TALERT4 */
#define MCP9600_DEVICE_ID_ADDRESS				0x20 /* Device ID/Revision register */

/*
 * Size of each Register in Bytes
 */
#define MCP9600_HOT_JUNC_TEMP_BYTES_COUNT		2U
#define MCP9600_DELTA_JUNC_TEMP_BYTES_COUNT		2U
#define MCP9600_COLD_JUNC_TEMP_BYTES_COUNT		2U
#define MCP9600_ADC_RAW_DATA_BYTES_COUNT		3U
#define MCP9600_STATUS_BYTES_COUNT              1U
#define MCP9600_SENSOR_CONFIG_BYTES_COUNT       1U
#define MCP9600_DEVICE_CONFIG_BYTES_COUNT		1U
#define MCP9600_ALERT1_CONFIG_BYTES_COUNT		1U
#define MCP9600_ALERT2_CONFIG_BYTES_COUNT		1U
#define MCP9600_ALERT3_CONFIG_BYTES_COUNT		1U
#define MCP9600_ALERT4_CONFIG_BYTES_COUNT		1U
#define MCP9600_ALERT1_HYSTERESIS_BYTES_COUNT	1U
#define MCP9600_ALERT2_HYSTERESIS_BYTES_COUNT	1U
#define MCP9600_ALERT3_HYSTERESIS_BYTES_COUNT	1U
#define MCP9600_ALERT4_HYSTERESIS_BYTES_COUNT	1U
#define MCP9600_ALERT1_LIMIT_BYTES_COUNT		2U
#define MCP9600_ALERT2_LIMIT_BYTES_COUNT		2U
#define MCP9600_ALERT3_LIMIT_BYTES_COUNT		2U
#define MCP9600_ALERT4_LIMIT_BYTES_COUNT		2U
#define MCP9600_DEVICE_ID_BYTES_COUNT			2U
/* a Parameter that holds the total number of bytes for all Internal Registers */
#define MCP9600_ALL_REGISTER_BYTES_COUNT		30U

/*
 * Bit Masks Used in THERMOCOUPLE TEMPERATURE REGISTER - TH (READ-ONLY)
 */
#define MCP9600_TH_SIGN					0x80
#define MCP9600_TH_POSITIVE				0x00
#define MCP9600_TH_NEGATIVE				0x80
#define MCP9600_TH_UPPER_BYTE_VAL_MASK	0x0F00
#define MCP9600_TH_LOWER_BYTE_VAL_MASK	0x00FF

/*
 * Bit Masks Used in HOT-JUNCTION TEMPERATURE REGISTER -TD (READ-ONLY)
 */
#define MCP9600_TD_SIGN					0x80
#define MCP9600_TD_POSITIVE				0x00
#define MCP9600_TD_NEGATIVE				0x80
#define MCP9600_TD_UPPER_BYTE_VAL_MASK	0x0F00
#define MCP9600_TD_LOWER_BYTE_VAL_MASK	0x00FF

/*
 * Bit Masks Used in COLD-JUNCTION TEMPERATURE REGISTER - TC (READ ONLY)
 */
#define MCP9600_TC_SIGN					0x80
#define MCP9600_TC_POSITIVE				0x00
#define MCP9600_TC_NEGATIVE				0x80
#define MCP9600_TC_UPPER_BYTE_VAL_MASK	0x0F00
#define MCP9600_TC_LOWER_BYTE_VAL_MASK	0x00FF

/*
 * Bit Masks Used in 24-BIT ADC REGISTER (READ-ONLY)
 */
#define MCP9600_ADC_DATA_SIGN			0x0000FC00
#define MCP9600_ADC_DATA_POSITIVE		0x00000000
#define MCP9600_ADC_DATA_NEGATIVE		0x0000FC00
#define MCP9600_ADC_DATA_BYTES_MASK     0x0003FFFF
/*
 * Bit Masks Used in STATUS REGISTER
 */
#define MCP9600_STATUS_BURST_MODE_CONV_FLAG		0X80
#define MCP9600_STATUS_TEMP_UPDATE_FLAG			0X40
#define MCP9600_STATUS_TEMP_RANGE_DETECT_FLAG	0X10
#define MCP9600_STATUS_ALERT4_STATUS_FLAG       0X08
#define MCP9600_STATUS_ALERT3_STATUS_FLAG       0X04
#define MCP9600_STATUS_ALERT2_STATUS_FLAG       0X02
#define MCP9600_STATUS_ALERT1_STATUS_FLAG       0X01

/*
 * Bit Masks Used in SENSOR CONFIGURATION REGISTER
 * Used as a Configuration Parameters
 */
#define MCP9600_THERMOCOUPLE_TYPE_K			    0X00
#define MCP9600_THERMOCOUPLE_TYPE_J			    0X10
#define MCP9600_THERMOCOUPLE_TYPE_T			    0X20
#define MCP9600_THERMOCOUPLE_TYPE_N			    0X30
#define MCP9600_THERMOCOUPLE_TYPE_S			    0X40
#define MCP9600_THERMOCOUPLE_TYPE_E			    0X50
#define MCP9600_THERMOCOUPLE_TYPE_B			    0X60
#define MCP9600_THERMOCOUPLE_TYPE_R			    0X70
#define MCP9600_THERMOCOUPLE_TYPE_MASK			0x70
#define MCP9600_THERMOCOUPLE_TYPE_OFFSET		4U

#define MCP9600_FILTER_COEFF_0					0X00
#define MCP9600_FILTER_COEFF_1					0X01
#define MCP9600_FILTER_COEFF_2					0X02
#define MCP9600_FILTER_COEFF_3					0X03
#define MCP9600_FILTER_COEFF_4					0X04
#define MCP9600_FILTER_COEFF_5					0X05
#define MCP9600_FILTER_COEFF_6					0X06
#define MCP9600_FILTER_COEFF_7					0X07
#define MCP9600_FILTER_COEFF_MASK				0x07

/*
 * Bit Masks Used in DEVICE CONFIGURATION REGISTER
 * Used as a Configuration Parameters
 */
#define MCP9600_DEVICE_CFG_COLD_JUNC_RES_025    0x80
#define MCP9600_DEVICE_CFG_COLD_JUNC_RES_00625	0x00
#define MCP9600_DEVICE_CFG_COLD_JUNC_RES_MASK	0x80
#define MCP9600_DEVICE_CFG_COLD_JUNC_RES_OFFSET	7U

#define MCP9600_DEVICE_CFG_ADC_RES_18_BIT		0x00
#define MCP9600_DEVICE_CFG_ADC_RES_16_BIT		0x20
#define MCP9600_DEVICE_CFG_ADC_RES_14_BIT		0x40
#define MCP9600_DEVICE_CFG_ADC_RES_12_BIT		0x60
#define MCP9600_DEVICE_CFG_ADC_RES_MASK			0x60
#define MCP9600_DEVICE_CFG_ADC_RES_OFFSET		5U

#define MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_1	  0x00
#define MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_2	  0x04
#define MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_4	  0x08
#define MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_8	  0x0C
#define MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_16	  0x10
#define MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_32	  0x14
#define MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_64	  0x18
#define MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_128	  0x1C
#define MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_MASK   0x1C
#define MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_OFFSET 2U

#define MCP9600_DEVICE_CFG_MODE_NORMAL_OPERATION 		0x00
#define MCP9600_DEVICE_CFG_MODE_SHUTDOWN_MODE			0x01
#define MCP9600_DEVICE_CFG_MODE_BURST_MODE				0x02
#define MCP9600_DEVICE_CFG_MODE_MASK					0x03
/*
 * Bit Masks Used in ALERT 1, 2, 3 AND 4 CONFIGURATION REGISTERS
 * Used as a Configuration Parameters
 */
#define MCP9600_ALERT_OUTPUT_ENABLE						0x01
#define MCP9600_ALERT_OUTPUT_DISABLE					0x00
#define MCP9600_ALERT_OUTPUT_STATE_MASK					0x01

#define MCP9600_ALERT_INTERRUPT_MODE					0x02
#define MCP9600_ALERT_COMPARATOR_MODE					0x00
#define MCP9600_ALERT_MODE_MASK							0x02
#define MCP9600_ALERT_MODE_OFFSET						1U

#define MCP9600_ALERT_OUTPUT_ACTIVE_HIGH_STATE			0x04
#define MCP9600_ALERT_OUTPUT_ACTIVE_LOW_STATE			0x00
#define MCP9600_ALERT_OUTPUT_ACTIVE_STATE_MASK			0x04
#define MCP9600_ALERT_OUTPUT_ACTIVE_STATE_OFFSET		2U

#define MCP9600_ALERT_LIMIT_FOR_RISING_TEMP				0x08
#define MCP9600_ALERT_LIMIT_FOR_FALLING_TEMP			0x00
#define MCP9600_ALERT_LIMIT_MASK						0x08
#define MCP9600_ALERT_LIMIT_OFFSET						3U

#define MCP9600_ALERT_MONITOR_TC_TEMP					0x10
#define MCP9600_ALERT_MONITOR_Th_TEMP					0x00
#define MCP9600_ALERT_MONITOR_TEMP_MASK					0x10
#define MCP9600_ALERT_MONITOR_TEMP_OFFSET				4U

#define MCP9600_ALERT_INTERRUPT_CLEAR					0x80
#define MCP9600_ALERT_NORMAL_OR_CLEARED_STATE			0x00
#define MCP9600_ALERT_INTERRUPT_MASK					0x80
#define MCP9600_ALERT_INTERRUPT_OFFSET					7U


/* Useful Masks Used in ALERT LIMIT and ALERT HYSTERESIS REGISTERS */
#define MCP9600_ALERT_MAX_POSITIVE_TEMP_LIMIT			2047
#define MCP9600_ALERT_MAX_NEGATIVE_TEMP_LIMIT		   (-2048)

#define MCP9600_ALERT_HYST_MAX_TEMP_LIMIT				1U
#define MCP9600_ALERT_HYST_MIN_TEMP_LIMIT				255U

 /*
  * Bit Masks Used in DEVICE ID AND REVISION ID REGISTER
  */
#define MCP9600_DEVICE_ID								0xFF00
#define MCP9600_MAJOR_REVISION_ID						0xF0
#define MCP9600_MINOR_REVISION_ID						0x0F


/* Configuration Parameters Used as IDs to discriminate between 4 Types of Alert and Hysteresis */
#define ALERT1											1U
#define ALERT2											2U
#define ALERT3											3U
#define ALERT4											4U

/***************************************** PUBLIC APIS PROTOTYPES ***********************************************/


/*****************************************************************************************************
 * Service name      : MCP9600_init                                                                  *
 * Parameters (in)   : None                                                                          *
 * Parameters (inout): None                                                                          *
 * Parameters (out)  : None                                                                          *
 * Return value      : MCP9600State_t                                                                *
 *                                    MCP9600_STATE_OK   : Module Initialized Successfully           *
 *                                    MCP9600_STATE_NOK  : Encounterd a problem during Initialization*
 *                                                                                                   *
 * Description       : Function Used to Initialize MCP9600 Module with all Pre-Compile Configurations*
 *                     Specified In MCP9600_cfg.h file after Selecting Wanted Configurations         *
 *****************************************************************************************************/
MCP9600State_t MCP9600_init(void);

/******************************************************************************************************
 * Service name      : MCP9600_readHotJuncTemp                                                        *
 * Parameters (in)   : None                                                                           *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : HotTemp        ->  Pointer to float value(contains temperature in degrees)     *
 * Return value      : MCP9600State_t                                                                 *
 *                                    MCP9600_STATE_OK   : Temperature Read Successfully              *
 *                                    MCP9600_STATE_NOK  : Encounterd a problem while reading Register*
 *                                                                                                    *
 * Description       : Function Used to read hot junction temperature by reading the corresponding    *
 *                     internal register and converting it to temperature in degrees celsius          *
 *                                                                                                    *
 ******************************************************************************************************/
MCP9600State_t MCP9600_readHotJuncTemp(f32* HotTemp);
/******************************************************************************************************
 * Service name      : MCP9600_readColdJuncTemp                                                       *
 * Parameters (in)   : None                                                                           *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : ColdTemp        ->  Pointer to float value(contains temperature in degrees)    *
 * Return value      : MCP9600State_t                                                                 *
 *                                    MCP9600_STATE_OK   : Temperature Read Successfully              *
 *                                    MCP9600_STATE_NOK  : Encounterd a problem while reading Register*
 *                                                                                                    *
 * Description       : Function Used to read cold/ambient junction temperature by reading the         *
 *                     corresponding internal register then converting it to temperature in           *                
 *                     degrees celsius                                                                *
 ******************************************************************************************************/
MCP9600State_t MCP9600_readColdJuncTemp(f32* ColdTemp);
/******************************************************************************************************
 * Service name      : MCP9600_readDeltaTemp                                                          *
 * Parameters (in)   : None                                                                           *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : DeltaTemp        ->  Pointer to float value(contains temperature in degrees)   *
 * Return value      : MCP9600State_t                                                                 *
 *                                    MCP9600_STATE_OK   : Temperature Read Successfully              *
 *                                    MCP9600_STATE_NOK  : Encounterd a problem while reading Register*
 *                                                                                                    *
 * Description       : Function Used to read Delta temperature by reading the corresponding internal  *
 *                     register then converting it to temperature in degrees celsius                  *                                                            *
 ******************************************************************************************************/
MCP9600State_t MCP9600_readDeltaTemp(f32* DeltaTemp);
/******************************************************************************************************
 * Service name      : MCP9600_readADCVal                                                             *
 * Parameters (in)   : None                                                                           *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : ADCVal        ->  Pointer to float value(contains ADC digital reading)         *
 * Return value      : MCP9600State_t                                                                 *
 *                                    MCP9600_STATE_OK   : ADC Value Read Successfully                *
 *                                    MCP9600_STATE_NOK  : Encounterd a problem while reading Register*
 * Description       : Function Used to read ADC Raw Data by reading the corresponding internal       *
 *                     Register                                                                       *
 ******************************************************************************************************/
MCP9600State_t MCP9600_readADCVal(f32* ADCVal);
/******************************************************************************************************
 * Service name      : MCP9600_getSensorStatus                                                        *
 * Parameters (in)   : None                                                                           *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : Status        ->  Pointer to MCP9600_SensorStatus_t Type                       *
 *                                      (contains status of sensors flag bits)                        *
 * Return value      : MCP9600State_t                                                                 *
 *                                    MCP9600_STATE_OK   : Sensor Status Read Successfully            *
 *                                    MCP9600_STATE_NOK  : Encounterd a problem while reading Register*
 * Description       : Function Used to read status of internal flag bits described                   *
 *                      in  STATUS REGISTER                                                           *
 ******************************************************************************************************/
MCP9600State_t MCP9600_getSensorStatus(MCP9600_SensorStatus_t* Status);
/******************************************************************************************************
 * Service name      : MCP9600_getDeviceInfo                                                          *
 * Parameters (in)   : None                                                                           *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : DeviceInfo        ->  Pointer to MCP9600_DeviceInfo_t Type                     *
 *                                          (contains Device Information)                             *
 * Return value      : MCP9600State_t                                                                 *
 *                                    MCP9600_STATE_OK   : Device Information Read Successfully       *
 *                                    MCP9600_STATE_NOK  : Encounterd a problem while reading Register*
 * Description       : Function Used to read Device Information described in  DEVICE ID AND           *
 *                     REVISION ID REGISTER such as Device ID, Major and Minor Revision Numbers       *
 ******************************************************************************************************/
MCP9600State_t MCP9600_getDeviceInfo(MCP9600_DeviceInfo_t* DeviceInfo);

/******************************************************************************************************
 * Service name      : MCP9600_setSensorConfig                                                        *
 * Parameters (in)   : ThermoType, FilterCoeff      ->  unsigned char Type                            *
 *                                                 (contains Thermocouple Type and Filter Coefficient)*
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : None                                                                           *
 * Return value      : MCP9600State_t                                                                 *
 *                            MCP9600_STATE_OK           : Sensor Configuration written Successfully  *
 *                            MCP9600_STATE_NOK          : Encounterd a problem while writing Register*
 *                            MCP9600_STATE_INVALID_PARAM: Invalid Input Parameter                    *
 * Description       : Function Used to Write Sensor Configurations to  SENSOR CONFIGURATION REGISTER *
 *                     which is Thermocouple Type and Filter Coefficient                              *
 ******************************************************************************************************/
MCP9600State_t MCP9600_setSensorConfig(u8 ThermoType, u8 FilterCoeff);
/******************************************************************************************************
 * Service name      : MCP9600_getSensorConfig                                                        *
 * Parameters (in)   : None                                                                           *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : ThermoType, FilterCoeff      ->  Pointer to unsigned char Type                 *
 *                                                 (contains Thermocouple Type and Filter Coefficient *
 * Return value      : MCP9600State_t                                                                 *
 *                            MCP9600_STATE_OK           : Sensor Configuration Read Successfully     *
 *                            MCP9600_STATE_NOK          : Encounterd a problem while reading Register*
 *                            MCP9600_STATE_INVALID_PARAM: Invalid Input Parameter                    *
 * Description       : Function Used to Read Sensor Configurations from SENSOR CONFIGURATION REGISTER *
 *                     which is Thermocouple Type and Filter Coefficient                              *
 ******************************************************************************************************/
MCP9600State_t MCP9600_getSensorConfig(u8* ThermoType, u8* FilterCoeff);

/******************************************************************************************************
 * Service name      : MCP9600_setDeviceConfig                                                        *
 * Parameters (in)   : ColdJuncResol, ADCResol, BurstSampleCount, Mode  ->  unsigned char Type        *
 *                                                                (contains all Device Configurations)*
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : None                                                                           *
 * Return value      : MCP9600State_t                                                                 *
 *                            MCP9600_STATE_OK           : Device Configuration written Successfully  *
 *                            MCP9600_STATE_NOK          : Encounterd a problem while writing Register*
 *                            MCP9600_STATE_INVALID_PARAM: Invalid Input Parameter                    *
 * Description       : Function Used to Write Device Configurations to  DEVICE CONFIGURATION REGISTER *
 *                     which is Device Mode, number of Samples in case of Burst Mode, Cold Junction   *
 *                     Temperature Resolution and ADC Resolution                                      *
 ******************************************************************************************************/
MCP9600State_t MCP9600_setDeviceConfig(u8 ColdJuncResol, u8 ADCResol, u8 BurstSampleCount, u8 Mode);
/******************************************************************************************************
 * Service name      : MCP9600_getDeviceConfig                                                        *
 * Parameters (in)   : None                                                                           *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : ColdJuncResol, ADCResol, BurstSampleCount, Mode ->Pointer to unsigned char Type*
 *                                                                (contains all Device Configurations)*
 * Return value      : MCP9600State_t                                                                 *
 *                            MCP9600_STATE_OK           : Device Configuration read Successfully     *
 *                            MCP9600_STATE_NOK          : Encounterd a problem while reading Register*
 *                            MCP9600_STATE_INVALID_PARAM: Invalid Input Parameter                    *
 * Description       : Function Used to Read Device Configurations to  DEVICE CONFIGURATION REGISTER  *
 *                     which is Device Mode, number of Samples in case of Burst Mode, Cold Junction   *
 *                     Temperature Resolution and ADC Resolution                                      *
 ******************************************************************************************************/
MCP9600State_t MCP9600_getDeviceConfig(u8* ColdJuncResol, u8* ADCResol, u8* BurstSampleCount, u8* Mode);

/******************************************************************************************************
 * Service name      : MCP9600_setDeviceConfig                                                        *
 * Parameters (in)   : AlertID      -> unsigned char Type         (contains Alert ID )                *
 *                     AlertConfig  -> MCP9600_AlertConfig_t Type (contains Alert Configurations)     *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : None                                                                           *
 * Return value      : MCP9600State_t                                                                 *
 *                            MCP9600_STATE_OK           : Alert Configuration written Successfully   *
 *                            MCP9600_STATE_NOK          : Encounterd a problem while writing Register*
 *                            MCP9600_STATE_INVALID_PARAM: Invalid Input Parameter                    *
 * Description       : Function Used to Write Alert Configurations to  ALERT CONFIGURATION REGISTERS  *
 *                     which is Alert Output Enable, Alert Mode, Active-High/Low Alert State,         *
 *                     Rise/Fall Alert Temperature Direction, Monitor TH or TC Temperature Detect     *
 *                     and Interrupt Clear for each of the 4 Alert Configuration Registers            *
 ******************************************************************************************************/
MCP9600State_t MCP9600_setAlertConfig(u8 AlertID, MCP9600_AlertConfig_t AlertConfig);
/******************************************************************************************************
 * Service name      : MCP9600_getAlertConfig                                                         *
 * Parameters (in)   : AlertID      -> unsigned char Type         (contains Alert ID )                *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : AlertConfig  -> Pointer to MCP9600_AlertConfig_t Type                          *
 *                                                                (contains Alert Configurations)     * 
 * Return value      : MCP9600State_t                                                                 *
 *                            MCP9600_STATE_OK           : Alert Configuration written Successfully   *
 *                            MCP9600_STATE_NOK          : Encounterd a problem while writing Register*
 *                            MCP9600_STATE_INVALID_PARAM: Invalid Input Parameter                    *
 * Description       : Function Used to Write Alert Configurations to  ALERT CONFIGURATION REGISTERS  *
 *                     which is Alert Output Enable, Alert Mode, Active-High/Low Alert State,         *
 *                     Rise/Fall Alert Temperature Direction, Monitor TH or TC Temperature Detect     *
 *                     and Interrupt Clear for each of the 4 Alert Configuration Register             *
 ******************************************************************************************************/
MCP9600State_t MCP9600_getAlertConfig(u8 AlertID, MCP9600_AlertConfig_t* AlertConfig);

/******************************************************************************************************
 * Service name      : MCP9600_setAlertLimit                                                          *
 * Parameters (in)   : AlertID         -> unsigned char Type      (contains Alert ID )                *
 *                     AlertLimitTemp  -> Float Type              (contains Alert Limit Temperature)  *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : None                                                                           *
 * Return value      : MCP9600State_t                                                                 *
 *                            MCP9600_STATE_OK           : Alert Limit Temp written Successfully      *
 *                            MCP9600_STATE_NOK          : Encounterd a problem while writing Register*
 *                            MCP9600_STATE_INVALID_PARAM: Invalid Input Parameter                    *
 * Description       : Function Used to Set Alert Limit Temperature for each of the 4 Alert Limtit    *
 *                     Registers                                                                      *
 ******************************************************************************************************/
MCP9600State_t MCP9600_setAlertLimit(u8 AlertID, f32 AlertLimitTemp);
/******************************************************************************************************
 * Service name      : MCP9600_getAlertLimit                                                          *
 * Parameters (in)   : AlertID         -> unsigned char Type      (contains Alert ID )                *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : AlertLimitTemp  -> Pointer to Float Type   (contains Alert Limit Temperature)  *
 * Return value      : MCP9600State_t                                                                 *
 *                            MCP9600_STATE_OK           : Alert Limit Temp Read Successfully         *
 *                            MCP9600_STATE_NOK          : Encounterd a problem while Reading Register*
 *                            MCP9600_STATE_INVALID_PARAM: Invalid Input Parameter                    *
 * Description       : Function Used to Read Alert Limit Temperature written to each of the 4 Alert   * 
 *                     Limtit Registers                                                               *
 ******************************************************************************************************/
MCP9600State_t MCP9600_getAlertLimit(u8 AlertID, f32* AlertLimitTemp);

/******************************************************************************************************
 * Service name      : MCP9600_setHystVal                                                             *
 * Parameters (in)   : AlertID         -> unsigned char Type   (contains Alert ID )                   *
 *                     HystVal         -> unsigned char Type   (contains Alert Limit Hysteresis Value)*
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : None                                                                           *
 * Return value      : MCP9600State_t                                                                 *
 *                            MCP9600_STATE_OK           : Alert Limit Hysteresis written Successfully*
 *                            MCP9600_STATE_NOK          : Encounterd a problem while writing Register*
 *                            MCP9600_STATE_INVALID_PARAM: Invalid Input Parameter                    *
 * Description       : Function Used to Set Alert Limit Hysteresis Value for each of the 4 Alert      *
 *                     Hysteresis Registers                                                           *
 ******************************************************************************************************/
MCP9600State_t MCP9600_setHystVal(u8 AlertID, u8 HystVal);
/******************************************************************************************************
 * Service name      : MCP9600_getHystVal                                                             *
 * Parameters (in)   : AlertID         -> unsigned char Type   (contains Alert ID )                   *
 * Parameters (inout): None                                                                           *
 * Parameters (out)  : AlertLimitTemp  -> Pointer to unsigned char Type                               *
 *                                                             (contains Alert Limit Hysteresis Value)*
 * Return value      : MCP9600State_t                                                                 *
 *                            MCP9600_STATE_OK           : Alert Limit Hysteresis Read Successfully   *
 *                            MCP9600_STATE_NOK          : Encounterd a problem while Reading Register*
 *                            MCP9600_STATE_INVALID_PARAM: Invalid Input Parameter                    *
 * Description       : Function Used to Read Alert Limit Hysteresis Value written to each of the      *
 *                     4 Alert Hysteresis Registers                                                   * 
 ******************************************************************************************************/
MCP9600State_t MCP9600_getHystVal(u8 AlertID, u8* HystVal);


#endif /* MCP9600_H_ */
