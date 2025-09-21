/*******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file, part of the VL53L8CX Ultra Lite Driver,
* is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
*******************************************************************************/

/***********************************/
/*  VL53L5CX ULD motion indicator  */
/***********************************/
/*
* This example shows the VL53L5CX motion indicator capabilities.
* To use this example, user needs to be sure that macro
* VL53L5CX_DISABLE_MOTION_INDICATOR is NOT enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_motion_indicator.h"

int example10(void)
{

	/*********************************/
	/*   VL53L5CX ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i;
	VL53L5CX_Configuration 	Dev;			/* Sensor configuration */
	VL53L5CX_Motion_Configuration 	motion_config;	/* Motion configuration*/
	VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */


	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Fill the platform structure with customer's implementation. For this
	* example, only the I2C address is used.
	*/
	Dev.platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;

	/* (Optional) Reset sensor toggling PINs (see platform, not in API) */
	//VL53L5CX_Reset_Sensor(&(Dev.platform));

	/* (Optional) Set a new I2C address if the wanted address is different
	* from the default one (filled with 0x20 for this example).
	*/
	//status = vl53l5cx_set_i2c_address(&Dev, 0x20);


	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L5CX sensor connected */
	status = vl53l5cx_is_alive(&Dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53L5CX not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53L5CX sensor */
	status = vl53l5cx_init(&Dev);
	if(status)
	{
		printf("VL53L5CX ULD Loading failed\n");
		return status;
	}

	printf("VL53L5CX ULD ready ! (Version : %s)\n",
			VL53L5CX_API_REVISION);


	/*********************************/
	/*   Program motion indicator    */
	/*********************************/

	/* Create motion indicator with resolution 4x4 */
	status = vl53l5cx_motion_indicator_init(&Dev, &motion_config, VL53L5CX_RESOLUTION_4X4);
	if(status)
	{
		printf("Motion indicator init failed with status : %u\n", status);
		return status;
	}

	/* (Optional) Change the min and max distance used to detect motions. The
	 * difference between min and max must never be >1500mm, and minimum never be <400mm,
	 * otherwise the function below returns error 127 */
	status = vl53l5cx_motion_indicator_set_distance_motion(&Dev, &motion_config, 1000, 2000);
	if(status)
	{
		printf("Motion indicator set distance motion failed with status : %u\n", status);
		return status;
	}

	/* If user want to change the resolution, he also needs to update the motion indicator resolution */
	//status = vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_4X4);
	//status = vl53l5cx_motion_indicator_set_resolution(&Dev, &motion_config, VL53L5CX_RESOLUTION_4X4);

	/* Increase ranging frequency for the example */
	status = vl53l5cx_set_ranging_frequency_hz(&Dev, 2);


	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53l5cx_start_ranging(&Dev);

	loop = 0;
	while(loop < 10)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A3
		 * (GPIO 1) when a new measurement is ready */

		status = vl53l5cx_check_data_ready(&Dev, &isReady);

		if(isReady)
		{
			vl53l5cx_get_ranging_data(&Dev, &Results);

			/* As the sensor is set in 4x4 mode by default, we have a total
			 * of 16 zones to print. For this example, only the data of first zone are
			 * print */
			printf("Print data no : %3u\n", Dev.streamcount);
			for(i = 0; i < 16; i++)
			{
				printf("Zone : %3d, Motion power : %3lu\n",
					i,
					Results.motion_indicator.motion[motion_config.map_id[i]]);
			}
			printf("\n");
			loop++;
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		VL53L5CX_WaitMs(&(Dev.platform), 5);
	}

	status = vl53l5cx_stop_ranging(&Dev);
	printf("End of ULD demo\n");
	return status;
}
