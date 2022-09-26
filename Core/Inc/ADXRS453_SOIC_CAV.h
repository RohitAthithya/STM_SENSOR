/*
 * ADXRS453_SOIC_CAV.h
 *
 *  Created on: 02-Sep-2022
*      	Author: Rohit Athithya V
 */
#include <stdint.h>
#include "stm32f4xx_hal.h"
#define ON 			1
#define OFF			0

#ifndef INC_ADXRS453_SOIC_CAV_H_
#define INC_ADXRS453_SOIC_CAV_H_

#ifndef __vo
	#define __vo volatile
#endif
#ifndef __co
	#define __co const
#endif



/********************************************************************************************************
 * 							SENSOR COMMAND & RESPONSE BIT DEFINITIONS
 *******************************************************************************************************/
#define ADXRS453_SLAVE_ID       				1
#define ADRXRS453_DIGITAL_OUT_SCALE_FACTOR		((float)0.0125) //it is 1/80
#define ADXRS453_NOT_STARTED 		            ((uint8_t)0x00)
#define ADXRS453_STARTED			            ((uint8_t)0XA0)
#define ADXRS453_SENSOR_AT_FAULT	            ((int16_t)0xFFFF)
#define GYRO_SENSOR_DATA_OFFSET					(140)
#define FAULT_BITS					            ((uint8_t)0x0F)

/********************************************************************************************************
 * 							MEMORY MAP FOR ADXRS435 REGISTERS
 *******************************************************************************************************/

#define MMRY_MAP_REG_RATE1			((uint8_t)0x00)	// Rate Registers
#define MMRY_MAP_REG_TEMP1			((uint8_t)0x02)	// Temperature Registers
#define MMRY_MAP_REG_LOCST1			((uint8_t)0x04)	// Low CST Memory Registers
#define MMRY_MAP_REG_HICST1			((uint8_t)0x06)	// High CST Memory Registers
#define MMRY_MAP_REG_QUAD1			((uint8_t)0x08)	// Quad Memory Registers
#define MMRY_MAP_REG_FAULT1			((uint8_t)0x0A)	// Fault Registers
#define MMRY_MAP_REG_PID1			((uint8_t)0x0C)	// Part ID Register 1
#define MMRY_MAP_REG_SNH			((uint8_t)0x0E)	// Serial Number Registers, 4 bytes - HIGH BYTES
#define MMRY_MAP_REG_SNL			((uint8_t)0x10)	// Serial Number Registers, 4 bytes - LOW BYTES


/********************************************************************************************************
 * 							DELAY TIMINGS
 *******************************************************************************************************/
#define ADXRS453_BOOTUP_DELAY					100
#define ADXRS453_SELFTEST_DELAY					50
#define ADXRS453_DATA_TRANSFER_TIMEOUT 			1000		//offer 1000 ms of timeout delay

/********************************************************************************************************
 * 							COMMAND AND RESPONSE STRUCTURE DEFINITIONS
 *******************************************************************************************************/

/*NOTE: DATA IS ALWAYS IN 2s COMPLEMENT FORM - SO ON PROCESSING MAKE SURE TO UTILISE (int16_t) explicit type-cast! */


/*
 * this structure is used to store the data related to ADXRS453data_read_command
 *
 * Usually, 0x20000003 is written to this structure and then transmitted during initiating sequence
 *
 * */
typedef union __CMD_SNSR_DATA_t
{
	__vo uint32_t cmd;
	struct data_field_of_command
	{
		__vo uint32_t P	 		    : 1	;
		__vo uint32_t CHK 		    : 1	;
		__co uint32_t RESERVED 	    : 26;
		__vo uint32_t SQ2 		    : 1	;
		__vo uint32_t DATA_ACCESS   : 1	;
		__vo uint32_t SQ0 		    : 1	;
		__vo uint32_t SQ1 		    : 1	;
	}
	cmd_bits;
}
CMD_SNSR_DATA_t;

/**
 * this structure is used to store the data related to ADXRS453register_read_command
 *
 */
typedef union __CMD_READ_t
{
	__vo uint32_t cmd;
	struct
	{
		__vo uint32_t P	 		           : 1	;
		__co uint32_t RESERVED 	           : 16	;// ALWAYS 0
		__vo uint32_t ADDRESS	           : 9	;
		__co uint32_t ADXRS453MODULE_BITS   : 3	;// HARDCODED AS 0b000
		__co uint32_t READ_CMD			   : 3	;// ALWAYS 0b100
	}
	cmd_bits;
}
CMD_READ_t;


/**
 * this structure is used to store the data related to ADXRS453register_write_command
 *
 */
typedef union __CMD_WRITE_t
{
	__vo uint32_t cmd;
	struct
	{
		__vo uint32_t P	 		           : 1	;
		__vo uint32_t DATA		           : 16	;// ALWAYS 0
		__vo uint32_t ADDRESS	           : 9	;
		__co uint32_t ADXRS453MODULE_BITS   : 3	;// HARDCODED AS 0b000
		__co uint32_t READ_CMD			   : 3	;// ALWAYS 0b010
	}
	cmd_bits;
}
CMD_WRITE_t;


/**
 * RESPONSE STRUCTURES
 */
typedef union __RSPNS_SENSOR_DATA_t
{
	__vo uint32_t response;
	struct
	{
		__vo uint32_t P1 					:	1;
		__vo uint32_t CHK					:	1;
		__vo uint32_t CST 					:	1;
		__vo uint32_t PWRR	 				:	1;
		__vo uint32_t POR 					:	1;
		__vo uint32_t NVM 					:	1;
		__vo uint32_t Q 					:	1;
		__vo uint32_t PLL 					:	1;
		__co uint32_t RESERVED				:	2;
		__vo uint32_t DATA 					:  16;
		__vo uint32_t ST					:	2;
		__vo uint32_t P0 					:	1;
		__vo uint32_t SQ 					:	3;
	}
	response_bits;
}
RSPNS_SENSOR_DATA_t;

/**
 * SENSOR RESPONSE DATA STRUCTURE TO STORE THE ADXRS453READ_COMMAND'S RESPONSE
 *
 */
typedef union __RSPNS_RW_t
{
	__vo uint32_t response;
	struct
	{
		__vo uint32_t P1 					:	1;
		__co uint32_t RESERVED				:	4;// 0b0000
		__vo uint32_t DATA 					:  16;
		__vo uint32_t ADXRS453MODULE_BITS	:	3;
		__co uint32_t RESERVED2				:	4;// 0b1110
		__vo uint32_t P0 					:	1;
		__co uint32_t RESERVED3				:	3;// 0b010
	}
	response_bits;
}
RSPNS_RW_t;

/**
 * error type response form the sensor
 */
typedef union __RSPNS_ERR_t
{
	__vo uint32_t response;
	struct
	{
		__vo uint32_t P1 					:	1;
		__vo uint32_t CHK					:	1;
		__vo uint32_t CST 					:	1;
		__vo uint32_t PWRR	 				:	1;
		__vo uint32_t POR 					:	1;
		__vo uint32_t NVM 					:	1;
		__vo uint32_t Q 					:	1;
		__vo uint32_t PLL 					:	1;
		__co uint32_t RESERVED				:	8;
		__vo uint32_t DU 					:	1;
		__vo uint32_t RE					:	1;
		__vo uint32_t SPI 					:	1;
		__co uint32_t RESERVED2				:	2;
		__vo uint32_t SM 					:	3;
		__co uint32_t RESERVED3				:	4;// 0b1110
		__vo uint32_t P0 					:	1;
		__co uint32_t RESERVED4				:	3;// 0b000
	}
	response_bits;
}
RSPNS_ERR_t;


/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/*function to change the endian-ness as per STM board!*/
//can also be used to set data at a particular pointer
static void bigToSmallEndian(uint32_t * pData, uint32_t data);
static void (*setData)(uint32_t * pData, uint32_t data) = bigToSmallEndian;

/*Fucntion to check the received fault data is as expected or not*/
static void checkFaultData(uint32_t);

/*! Sets or clears the parity bit in order to ensure that the overall parity of
the data word is odd. */
uint8_t ADXRS453_findParityBit(uint32_t data);


/*! Reads temperature from ADXRS453 and converts it to degrees Celsius. */
uint8_t ADXRS453_getTemperature(void);



/*Execute the start up sequence recommended by the sensor datasheet*/
/* @retval: 0xAA : in-case the sensor was started and initialisation sequence was
 * successfully done, sensor now ready to transmit sensor data*/
uint8_t ADXRS453_startUpSequence(void);

/*Following functions are for sending commands from master to ADXRS453*/
/*SPI - Command : Sensor data*/
int16_t ADXRS453_cmd_sensorData(uint32_t data);

/*SPI - Command : Read from register of ADXRS453*/
uint16_t ADXRS453_cmd_readFromAddress(uint16_t addr);

/*SPI - Command : Write to register of ADXRS453*/
void ADXRS453_cmd_writeToAddress(uint16_t addr, uint16_t data);

/*SPI - Response : check the response of the data received,
 * so as to segregate the values as per command sent
 * */
void ADXRS453_rspns_checkData(void);

/*SPI - Response : Sensor data */
void ADXRS453_rsnps_sensorData(void);

/*SPI - Response : Reply to read command with the data from the register*/
void ADXRS453_rspns_returnDataAfterRead(void);

/*SPI - Response : Reply to write command with the data from the register*/
void ADXRS453_rspns_returnDataAfterWrite(void);

/*SPI - Response : Reply to read command with the error data from the register*/


/*32 bit Tx-Rx SPI function*/
HAL_StatusTypeDef SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
		uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);



/* Program to decode the sensor data from the gyroscope to angular speed of degree/second */
float ADXRS453_decode_sensor_Data(int16_t *);
#endif /* INC_ADXRS453_SOIC_CAV_H_ */
