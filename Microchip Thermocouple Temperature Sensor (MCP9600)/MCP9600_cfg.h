/********************************************************************************************
 *                                                                                          *
 * File Name   : MCP9600_cfg.h                                                              *
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
 *               configuration file that contains needed configuration Parameters.          *
 *                                                                                          *
 ********************************************************************************************/

#ifndef MCP9600_CFG_H_
#define MCP9600_CFG_H_

/*
 * Select the MCP9600 Thermocouple Type
 * Options:-
 * 			MCP9600_THERMOCOUPLE_TYPE_K
			MCP9600_THERMOCOUPLE_TYPE_J
			MCP9600_THERMOCOUPLE_TYPE_T
			MCP9600_THERMOCOUPLE_TYPE_N
			MCP9600_THERMOCOUPLE_TYPE_S
			MCP9600_THERMOCOUPLE_TYPE_E
			MCP9600_THERMOCOUPLE_TYPE_B
            MCP9600_THERMOCOUPLE_TYPE_R
 */
#define  MCP9600_THERMOCOUPLE_TYPE			MCP9600_THERMOCOUPLE_TYPE_K

/*
 * Select the MCP9600 Filter Coefficient
 * Options:-
 *          MCP9600_FILTER_COEFF_0
            MCP9600_FILTER_COEFF_1
            MCP9600_FILTER_COEFF_2
            MCP9600_FILTER_COEFF_3
            MCP9600_FILTER_COEFF_4
            MCP9600_FILTER_COEFF_5
            MCP9600_FILTER_COEFF_6
            MCP9600_FILTER_COEFF_7
 */
#define  MCP9600_FILTER_COEFF				MCP9600_FILTER_COEFF_0

/*
 * Select MCP9600 Temperature Sensor I2C Address Levels Which specified by A2, A1, A0 Hardware Address Bits
 * Levels 1,2,3,4,5,6 Can be Obtained by connecting ADDR Pin to Voltage Divider Network with
 * Resistance values as described in DataSheet
 * Options:-
			MCP9600_I2C_ADDRESS_LEVEL_0
			MCP9600_I2C_ADDRESS_LEVEL_1
			MCP9600_I2C_ADDRESS_LEVEL_2
			MCP9600_I2C_ADDRESS_LEVEL_3
			MCP9600_I2C_ADDRESS_LEVEL_4
			MCP9600_I2C_ADDRESS_LEVEL_5
			MCP9600_I2C_ADDRESS_LEVEL_6
			MCP9600_I2C_ADDRESS_LEVEL_7
 */
#define MCP9600_I2C_ADDRESS_LEVEL			MCP9600_I2C_ADDRESS_LEVEL_0

/*
 * The Whole MCP9600 Temperature Sensor I2C Slave Address
 * as 0b1100 concatenated with selected address level
 */
#define MCP9600_SENSOR_I2C_ADDRESS     (u8)(MCP9600_I2C_FIXED_ADDRESS_PART | MCP9600_I2C_ADDRESS_LEVEL)

/*
 * Select Cold-Junction/Ambient Sensor Resolution
 * Options:-
  			MCP9600_DEVICE_CFG_COLD_JUNC_RES_025
			MCP9600_DEVICE_CFG_COLD_JUNC_RES_00625
 */
#define MCP9600_COLD_JUNC_RESOLUTION		MCP9600_DEVICE_CFG_COLD_JUNC_RES_00625

/*
 * Select ADC Measurement Resolution
 * OPtions:-
 	 	 	MCP9600_DEVICE_CFG_ADC_RES_18_BIT
			MCP9600_DEVICE_CFG_ADC_RES_16_BIT
			MCP9600_DEVICE_CFG_ADC_RES_14_BIT
            MCP9600_DEVICE_CFG_ADC_RES_12_BIT
 */
#define MCP9600_ADC_RESOLUTION				MCP9600_DEVICE_CFG_ADC_RES_18_BIT

/*
 * Select MCP9600 Mode
 * Options:-
  			MCP9600_DEVICE_CFG_MODE_NORMAL_OPERATION
			MCP9600_DEVICE_CFG_MODE_SHUTDOWN_MODE
			MCP9600_DEVICE_CFG_MODE_BURST_MODE
 */
#define MCP9600_MODE						MCP9600_DEVICE_CFG_MODE_NORMAL_OPERATION
/*
 * Select Burst Mode Temperature Samples Number
 * Options:-
 	 	 	MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_1
			MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_2
			MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_4
			MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_8
			MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_16
			MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_32
			MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_64
			MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_128
 */
#if MCP9600_MODE == MCP9600_DEVICE_CFG_MODE_BURST_MODE

	#define MCP9600_BURST_MODE_TEMP_SAMPLES		MCP9600_DEVICE_CFG_BURST_MODE_TEMP_SAMPLES_1

#endif

/*
 * Enable/Disable Alarm1,2,3 and 4
 * Options:-
 	 	 	 MCP9600_ALERT_OUTPUT_ENABLE
			 MCP9600_ALERT_OUTPUT_DISABLE
 */
#define MCP9600_ALERT1_OUTPUT_STATE		         MCP9600_ALERT_OUTPUT_DISABLE
#define MCP9600_ALERT2_OUTPUT_STATE              MCP9600_ALERT_OUTPUT_ENABLE
#define MCP9600_ALERT3_OUTPUT_STATE              MCP9600_ALERT_OUTPUT_DISABLE
#define MCP9600_ALERT4_OUTPUT_STATE              MCP9600_ALERT_OUTPUT_ENABLE


/*
 * If Alarm1,2,3 and 4 is Enabled, Do the following for the Enbled Alarm...
 *
 * Select Alarm1,2,3 and 4 Mode
 * Options:-
 	 	 	 MCP9600_ALERT_INTERRUPT_MODE
			 MCP9600_ALERT_COMPARATOR_MODE

 * Select Alarm1,2,3 and 4 Output Level
 * Options:-
  			MCP9600_ALERT_OUTPUT_ACTIVE_HIGH_STATE
			MCP9600_ALERT_OUTPUT_ACTIVE_LOW_STATE

 * Select Alarm1,2,3 and 4 Temperature Direction
 * Options:-
 	 	 	 MCP9600_ALERT_LIMIT_FOR_RISING_TEMP
			 MCP9600_ALERT_LIMIT_FOR_FALLING_TEMP

 * Select Alarm1,2,3 and 4 Temperature Monitoring
 * Options:-
 	 	 	 MCP9600_ALERT_MONITOR_TC_TEMP
		     MCP9600_ALERT_MONITOR_Th_TEMP

 * Select Value of Alarm and Hysteresis Temperature

 */
#if MCP9600_ALERT1_OUTPUT_STATE == MCP9600_ALERT_OUTPUT_ENABLE

	#define MCP9600_ALERT1_MODE			    MCP9600_ALERT_INTERRUPT_MODE
	#define MCP9600_ALERT1_OUTPUT_LEVEL     MCP9600_ALERT_OUTPUT_ACTIVE_LOW_STATE
	#define MCP9600_ALERT1_TEMP_DIR			MCP9600_ALERT_LIMIT_FOR_RISING_TEMP
	#define MCP9600_ALERT1_TEMP_MONITOR		MCP9600_ALERT_MONITOR_TC_TEMP

#endif

#if MCP9600_ALERT2_OUTPUT_STATE == MCP9600_ALERT_OUTPUT_ENABLE

	#define MCP9600_ALERT2_MODE			    MCP9600_ALERT_INTERRUPT_MODE
	#define MCP9600_ALERT2_OUTPUT_LEVEL     MCP9600_ALERT_OUTPUT_ACTIVE_LOW_STATE
	#define MCP9600_ALERT2_TEMP_DIR			MCP9600_ALERT_LIMIT_FOR_RISING_TEMP
	#define MCP9600_ALERT2_TEMP_MONITOR		MCP9600_ALERT_MONITOR_TC_TEMP

#endif

#if MCP9600_ALERT3_OUTPUT_STATE == MCP9600_ALERT_OUTPUT_ENABLE

	#define MCP9600_ALERT3_MODE			    MCP9600_ALERT_INTERRUPT_MODE
	#define MCP9600_ALERT3_OUTPUT_LEVEL     MCP9600_ALERT_OUTPUT_ACTIVE_LOW_STATE
	#define MCP9600_ALERT3_TEMP_DIR			MCP9600_ALERT_LIMIT_FOR_RISING_TEMP
	#define MCP9600_ALERT3_TEMP_MONITOR		MCP9600_ALERT_MONITOR_TC_TEMP

#endif

#if MCP9600_ALERT4_OUTPUT_STATE == MCP9600_ALERT_OUTPUT_ENABLE

	#define MCP9600_ALERT4_MODE			    MCP9600_ALERT_INTERRUPT_MODE
	#define MCP9600_ALERT4_OUTPUT_LEVEL     MCP9600_ALERT_OUTPUT_ACTIVE_LOW_STATE
	#define MCP9600_ALERT4_TEMP_DIR			MCP9600_ALERT_LIMIT_FOR_RISING_TEMP
	#define MCP9600_ALERT4_TEMP_MONITOR		MCP9600_ALERT_MONITOR_TC_TEMP

#endif


#endif /* MCP9600_CFG_H_ */
