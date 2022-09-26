/*
 * ADXRS453_SOIC_CAV.c
 *
 *  Created on: 03-Sep-2022
 *      Author: Rohit Athithya V
 */

//GCC supports only two’s complement integer types, and all bit patterns are ordinary values.
// https://gcc.gnu.org/onlinedocs/gcc/Integers-implementation.html
#include "ADXRS453_SOIC_CAV.h"
#include "stm32f4xx_hal.h"
/*
 * sensor_status:
 * 0x00 => Not booted yet
 * 0xaf => received the intial_data: 0x0000 0001
 * 0x0f => fault bits correctly received
 * 0xa0 => RATE transfer mode
 * 0xff => sensor_at_fault_status or self_test_status
 */
static uint8_t sensor_status = 0;
extern uint8_t LAST_READ;

static void bigToSmallEndian(uint32_t * pData, uint32_t data)
{
	*pData = data >> 16;
	*pData |= data << 16;
}

static void checkFaultData(uint32_t data)
{
	if(((data & 0xFF) == 0xFF) || ((data & 0xFE) == 0xFE) )
		sensor_status = 0x0f;
	if(((data & 0b1111) == 0b0001) || ((data & 0b1111) == 0b0000))
		sensor_status = ADXRS453_STARTED;

}

uint8_t ADXRS453_findParityBit(uint32_t data)
{
	uint8_t sum = 0;									//to count the number of ones in the data
	for (register uint16_t i = 0; i < 32; ++i)			//sum all bits - to see no. of ones present
		sum += ((data >> i)&((uint32_t)0b1));


	if(!(sum & (uint8_t)0b1))							//if sum is not odd
		return (0b1);									//return 1/0, so that the data parity is odd
	return (0b0);
}


uint8_t ADXRS453_getTemperature(void)
{
	(void)0x02000000;
}


uint8_t ADXRS453_startUpSequence(void)
{
	extern SPI_HandleTypeDef hspi2;
	extern uint32_t cmd_data;
	extern uint32_t gyro_data;

//1. power up the device
/*Implement in the main code: to turn on the SPI phrl. on the syst. bootup itself*/

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 , GPIO_PIN_SET);//mark the initiation of startup sequence

//2. wait for >=100ms for the device to bootup
	HAL_Delay(ADXRS453_BOOTUP_DELAY);//NOT REQUIRED IF SENSOR WAS TURN ON ALONG WITH uC - so 100ms would have been done already

//3. transmit data: 0x20000003 (SQ0 = 0, SQ1 = 0 SQ2 = 0, CHK = 1, P = 1 to make the data an odd pair)
	setData(&cmd_data, 0x20000003);
	//transmitting 2, 16 bits of data => the size of the transmission is 2
	SPI_TransmitReceive(&hspi2, (uint8_t *)&cmd_data, (uint8_t *)&gyro_data, 2, 1000);
	bigToSmallEndian(&gyro_data, gyro_data);//see its pass by value so no issues

//4.check if the current gyro_data == initial command value (should be : 0x00000001)

	if(gyro_data == 0x00000001)
		sensor_status = 0xAF;
	//then proceed


//5. >=50ms delay to allow the sensor module to generate necessary internal fault generation
	HAL_Delay(ADXRS453_SELFTEST_DELAY);

//6. transmit data: 0x20000000 (SQ0 = 0, SQ1 = 0 SQ2 = 0, CHK = 0, P = 0)
//and receive a dummy byte - in next receive we shall obtain the fault values
	setData(&cmd_data, 0x20000000);
	SPI_TransmitReceive(&hspi2, (uint8_t *)&cmd_data, (uint8_t *)&gyro_data, 2, 1000);
	bigToSmallEndian(&gyro_data, gyro_data);

//7. >=50ms  delay to allow sensor module to clear all the faults
	HAL_Delay(ADXRS453_SELFTEST_DELAY);

//8. transmit data: 0x20000000 (SQ0 = 0, SQ1 = 0 SQ2 = 0, CHK = 0, P = 0)
//and receive the fault condition values - to see what faults were checked internally!
//in this data from the MISO pin, we can see that ST bits are 10 => indicating self-test data
	setData(&cmd_data, 0x20000000);
	SPI_TransmitReceive(&hspi2, (uint8_t *)&cmd_data, (uint8_t *)&gyro_data, 2, 1000);
	bigToSmallEndian(&gyro_data, gyro_data);
	checkFaultData(gyro_data);

	if(sensor_status != 0x0f)
		goto fault;
//9. delay for t(TD) (usually, > 0.1us) (100us, chosen in our project)
	//need to work on this

//10. transmit data: 0x20000000 (SQ0 = 0, SQ1 = 0 SQ2 = 0, CHK = 0, P = 0)
//and receive in MISO , the same fault bit (because the new data will be available only after CS goes high!)
	setData(&cmd_data, 0x20000000);
	SPI_TransmitReceive(&hspi2, (uint8_t *)&cmd_data, (uint8_t *)&gyro_data, 2, 1000);
	bigToSmallEndian(&gyro_data, gyro_data);
	checkFaultData(gyro_data);
	if(sensor_status != 0x0f)
		goto fault;
//11. assert that the device it ready for data transmission. and return ctrl to caller
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	sensor_status = 0xA0;
	return 0x01;

fault:
	return 0x00;
}


int16_t ADXRS453_cmd_sensorData(uint32_t data)
{
	extern uint32_t gyro_data;
	extern uint32_t cmd_data;

	//RSPNS_SENSOR_DATA_t *sensor_data = &gyro_data;
	//CMD_SNSR_DATA_t *mstr_data = &cmd_data;


	//if(sensor_status == ADXRS453_STARTED)
	{
		//1.write the sensor_data_read command using the corr. structure
		extern SPI_HandleTypeDef hspi2;

		setData(&cmd_data, data);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

		SPI_TransmitReceive(&hspi2, (uint8_t *)&cmd_data, (uint8_t *)&gyro_data, 2, 1000);
		bigToSmallEndian(&gyro_data, (uint32_t)gyro_data);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

		//2. check if all the fault bits are proper - i.e., indiacte NO_FAULT
		//also, based on the corresponding bits assign the corr. structure to the data
		checkFaultData(gyro_data);

		if(((gyro_data >> 29) & 0b111) == 0b000)
		{
			//if st bits = 01 => valid sensor data
			if(( (gyro_data >> 26) & 0b11) == 0b01)
			{
				int16_t temp = (int16_t)(((RSPNS_SENSOR_DATA_t *)(&gyro_data))->response_bits.DATA);
				return temp;
			}
			if(( (gyro_data >> 24) & 0b1111) == 0b1110)
			{
				//r/w error => return 0,
				return 0x00;
			}

		}


	}

}



HAL_StatusTypeDef SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout)
{
	  uint32_t             tmp_mode;
	  HAL_SPI_StateTypeDef tmp_state;
	  uint32_t             tickstart;

	  /* Variable used to alternate Rx and Tx during transfer */
	  uint32_t             txallowed = 1U;
	  HAL_StatusTypeDef    errorcode = HAL_OK;

	  /* Check Direction parameter */
	  assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

	  /* Process Locked */
	  __HAL_LOCK(hspi);

	  /* Init tickstart for timeout management*/
	  tickstart = HAL_GetTick();

	  /* Init temporary variables */
	  tmp_state           = hspi->State;
	  tmp_mode            = hspi->Init.Mode;
	  //initial_TxXferCount = Size;

	  if (!((tmp_state == HAL_SPI_STATE_READY) || \
	        ((tmp_mode == SPI_MODE_MASTER) && (hspi->Init.Direction == SPI_DIRECTION_2LINES) && (tmp_state == HAL_SPI_STATE_BUSY_RX))))
	  {
		errorcode = HAL_BUSY;
		goto error;
	  }

	  if ((pTxData == NULL) || (pRxData == NULL) || (Size == 0U))
	  {
	    errorcode = HAL_ERROR;
	    goto error;
	  }

	  /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
	  if (hspi->State != HAL_SPI_STATE_BUSY_RX)
	  {
	    hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
	  }

	  /* Set the transaction information */
	  hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
	  hspi->pRxBuffPtr  = (uint8_t *)pRxData;
	  hspi->RxXferCount = Size;
	  hspi->RxXferSize  = Size;
	  hspi->pTxBuffPtr  = (uint8_t *)pTxData;
	  hspi->TxXferCount = Size;
	  hspi->TxXferSize  = Size;

	  /*Init field not used in handle to zero */
	  hspi->RxISR       = NULL;
	  hspi->TxISR       = NULL;

	  /* Check if the SPI is already enabled */
	  if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	  {
	    /* Enable SPI peripheral */
		  __HAL_SPI_ENABLE(hspi);
	  }

	  /* Transmit and Receive data in 16 Bit mode */

	while ((hspi->TxXferCount > 0U) || (hspi->RxXferCount > 0U))
	{
	  /* Check TXE flag */
	  if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) && (hspi->TxXferCount > 0U) && (txallowed == 1U))
	  {
		hspi->Instance->DR = *((uint16_t *)hspi->pTxBuffPtr);
		hspi->pTxBuffPtr += sizeof(uint16_t);
		hspi->TxXferCount--;
		/* Next Data is a reception (Rx). Tx not allowed */
		txallowed = 0U;
	  }

	  /* Check RXNE flag */
	  if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) && (hspi->RxXferCount > 0U))
	  {
		*((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DR;
		hspi->pRxBuffPtr += sizeof(uint16_t);
		hspi->RxXferCount--;
		/* Next Data is a Transmission (Tx). Tx is allowed */
		txallowed = 1U;
	  }
	  if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)) && (hspi->TxXferCount > 0U) && (txallowed == 1U))
	  {
		hspi->Instance->DR = *((uint16_t *)hspi->pTxBuffPtr);
		hspi->pTxBuffPtr += sizeof(uint16_t);
		hspi->TxXferCount--;
		/* Next Data is a reception (Rx). Tx not allowed */
		txallowed = 0U;
	  }

	  /* Check RXNE flag */
	  if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)) && (hspi->RxXferCount > 0U))
	  {
		*((uint16_t *)hspi->pRxBuffPtr) = (uint16_t)hspi->Instance->DR;
		hspi->pRxBuffPtr += sizeof(uint16_t);
		hspi->RxXferCount--;
		/* Next Data is a Transmission (Tx). Tx is allowed */
		txallowed = 1U;
	  }
	  if (((HAL_GetTick() - tickstart) >=  Timeout) && (Timeout != HAL_MAX_DELAY))
	  {
		errorcode = HAL_TIMEOUT;
		goto error;
	  }
	}


	  /* Clear overrun flag in 2 Lines communication mode because received is not read */
	  if (hspi->Init.Direction == SPI_DIRECTION_2LINES)
	  {
	    __HAL_SPI_CLEAR_OVRFLAG(hspi);
	  }

	error :
	  hspi->State = HAL_SPI_STATE_READY;
	  __HAL_UNLOCK(hspi);
	  //user addition over here
	  __HAL_SPI_DISABLE(hspi);

	  return errorcode;

}


inline float ADXRS453_decode_sensor_Data(int16_t * pData)
{
	return (  *pData * (float)(ADRXRS453_DIGITAL_OUT_SCALE_FACTOR)  );
}
