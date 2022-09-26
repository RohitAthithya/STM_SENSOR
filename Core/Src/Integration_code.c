/*
 * Integration_code.c
 *
 *  Created on: 26-Sep-2022
 *      Author: ADMIN
 */

#include <main.h>


int reading_after_first_dummy_read = 0;
__vo CMD_SNSR_DATA_t mstr_cmd_data = {0};

float data_at_k = 0;
float data_at_k_1 = 0;
float dummy_read;

float time_value_for_integration = 100 * (0.000001);//us
float pos = 0;

int main()
{
	mstr_cmd_data.cmd_bits.DATA_ACCESS 	= SET;
	mstr_cmd_data.cmd_bits.CHK 			= RESET;
	mstr_cmd_data.cmd_bits.P 			= 0b1 & ADXRS453_findParityBit(mstr_cmd_data.cmd);
	ADXRS453_startUpSequence();

	dummy_read = ADXRS453_decode_sensor_Data(ADXRS453_cmd_sensorData(mstr_cmd_data.cmd));
	reading_after_first_dummy_read = 0b1;

	//timer start
	//-------100us counted and the interrupt is generated




}





int interrupt_handler()
{

	if(reading_after_first_dummy_read == 1)
	{
		reading_after_first_dummy_read = 0;
		data_at_k_1 = 0;
	}
	else
	{
		data_at_k_1 = data_at_k;
	}
	data_at_k = ADXRS453_decode_sensor_Data(ADXRS453_cmd_sensorData(mstr_cmd_data.cmd));

	pos += ((data_at_k + data_at_k_1)/2) * time_value_for_integration;


}
