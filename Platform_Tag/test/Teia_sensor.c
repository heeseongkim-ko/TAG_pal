
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h> 
#include <stdlib.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
//#include "nrfx_spi.h"
//#include "nrfx_spim.h"
#include "nrf_uart.h"
#include "app_uart.h"
#include "nrf_drv_twi.h"
#include "def_var.h"
#include "LIS2DH12.h"
#include "NT3H2111.h"
#include "non_volatile_memory.h"
#include "UWB_routines.h"
/**
 * @brief Check which sensors are mounted on the tag and set it into standby mode, because of energy saving
 */
void sens_check_and_standby(void)
{
    //initialize sensors mounted flags//
    sensors_params.ACC_mounted = 0;
    sensors_params.GYRO_mounted = 0;
    sensors_params.MAG_mounted = 0;
    sensors_params.BARO_mounted = 0;


    //check if sensors are mounted on the tag and set that to low power modes//

    if(lis2dh12_test())                             //check if accelerometer chip les2dh12 is mounted and alive
	{
		sensors_params.ACC_mounted |= LIS2DH12;     //set the flag - accelerometer lis2dh2 is mounted on the tag
		lis2dh12_init();                            //initialize the sensor
		lis2dh12_set_Standby();						//set sensor to low power mode
	}

/*
	if (mpu9250_test())							    //check if sensor MPU9250 is mounted and alive
	{
		sensors_params.ACC_mounted |= MPU9250;      //set the flag - accelerometer lis2dh2 is mounted on the tag
		sensors_params.GYRO_mounted = MPU9250;

		mpu9250_WOM(LP_ODR_0_24,4); 				//very low power mode of accelerometer
		mpu9250_dis_gyro();							// shutdown gyroscope
		mpu9250_disable();

        mpu9250_bypass_I2C();						//connect primary and auxiliary I2Cs on MPU9250 together - the sensor ak8963 is connected to aux I2C
		delay_and_sleep(1000,false,false);			//wait while I2Cs not connected together

        if (AK8963_test())							//check if magnetometer chip AK8963 is mounted and alive
        {
            sensors_params.MAG_mounted = MPU9250;   //set the flag - magnetometer AK8963 (part of MPU9250) is mounted on the tag
            mpu9250_dis_mag();
        }
    }
*/
/*
	if(dps310_test())                                //check if barometer chip dps310 is mounted and alive
	{
		sensors_params.BARO_mounted = true;          //set the flag - barometer dps310 is mounted on the tag
		dps310_reset();                              //reset barometer - chip error workaround
		delay_and_sleep(10,true,true);               //wait until sensors startups
		dps310_set_standby();                        //set sensor to low power state
		dps310_read_coefs();                         //read calibration coefficients from sensor
		set_baro_wake_up_timestamp(get_RTC_timestamp(true));
	}
*/
}


/**
 * @brief Configure all mounted sensors according to user setting.
 */

void sensors_config(void)
{

#ifdef ACCELEROMETER_IN_USE
	if (sensors_params.motion_control_mode)
	{
		if (sensors_params.ACC_mounted & LIS2DH12)
		{
			ACC_IRQ_disable();																	        //disable IRQ from ACC while setting

			uint16_t threshold = (sensors_params.wakeup_threshold & 0x3FFFUL);   // mask out 2 highest bits and convert it to sensor required value

            uint8_t ODR;
			if(tdoaParameters.refresh_rate_ms <= 20) ODR = LIS2DH12_ODR200_Hz;
			else if(tdoaParameters.refresh_rate_ms <  40) ODR = LIS2DH12_ODR100_Hz;
			else if(tdoaParameters.refresh_rate_ms <  80) ODR = LIS2DH12_ODR50_Hz;
			else ODR = LIS2DH12_ODR25_Hz;


			lis2dh12_WOM_setting(ODR,threshold,LIS2DH12_HP_LIGHT);		                //set acc sampling rate and threshold of acceleration above which tag will be in sleep mode
			lis2dh12_get_int_src();                                                                     //clear interrupt flags in chip
		}
	}
	else
	{
		ACC_IRQ_disable();
        if(sensors_params.ACC_mounted & LIS2DH12) lis2dh12_set_Standby();                        //disable sensors - energy saving
	}
#endif
/*
	if((sensors_params.EBlink_cont &(EBLINK_CONT_ACC | EBLINK_CONT_MAG | EBLINK_CONT_GYRO)) && sensors_params.GYRO_mounted)
 	{
		if(sensors_params.AHRS_enable)mpu9250_init_all();			//init and enable all sensors in MPU
		else
		{
			if(sensors_params.EBlink_cont &EBLINK_CONT_ACC) mpu9250_init_acc();
			if(sensors_params.EBlink_cont &EBLINK_CONT_MAG) mpu9250_init_mag();
			if(sensors_params.EBlink_cont &EBLINK_CONT_GYRO)mpu9250_init_gyro();
		}
	}
        
	if(sensors_params.AHRS_enable)
	{
		if(sensors_params.ACC_mounted & MPU9250)
		{
		    mpu9250_init_all();
			mpu9250_set_full_scales(GYRO_FS_1000dps, ACC_FS_8g);		    //set full scale of accelerometer and gyroscope

			sensors_data.gyro_offsets_SC.x_axis = (sensors_data.gyro_offsets.x_axis + (1 << 1)) >> 2;
			sensors_data.gyro_offsets_SC.y_axis = (sensors_data.gyro_offsets.y_axis + (1 << 1)) >> 2;
			sensors_data.gyro_offsets_SC.z_axis = (sensors_data.gyro_offsets.z_axis + (1 << 1)) >> 2;

			sensors_data.acc_offsets_SC.x_axis = (sensors_data.acc_offsets.x_axis + (1 << 1)) >> 2;
			sensors_data.acc_offsets_SC.y_axis = (sensors_data.acc_offsets.y_axis + (1 << 1)) >> 2;
			sensors_data.acc_offsets_SC.z_axis = (sensors_data.acc_offsets.z_axis + (1 << 1)) >> 2;
		}
		else if(sensors_params.ACC_mounted & LIS2DH12)
		{
			lis2dh12_set_ODR(LIS2DH12_ODR25_Hz);
			lis2dh12_set_FS(LIS2DH12_FS_8g);
			lis2dh12_set_all_axes_active();                                             //all accelerometer axes should be measured
			lis2dh12_set_precision(LIS2DH12_HR_MODE);
			lis2dh12_set_Active();
		}

		if(sensors_params.AHRS_enable == 3) dps310_init(sensors_params.BARO_setting);
		else							    dps310_set_standby();
	}
	else if((sensors_params.EBlink_cont & (EBLINK_CONT_ACC | EBLINK_CONT_GYRO)))
	{
		if(sensors_params.ACC_mounted & MPU9250)
		{
			mpu9250_set_full_scales(sensors_params.gyro_FS<<3, sensors_params.acc_FS<<3);    //set full scale of accelerometer and gyroscope - that is possible only for raw data
			switch(sensors_params.gyro_FS<<3)
			{
				case (GYRO_FS_2000dps):
					sensors_data.gyro_offsets_SC.x_axis = (sensors_data.gyro_offsets.x_axis + (1 << 2)) >> 3;
					sensors_data.gyro_offsets_SC.y_axis = (sensors_data.gyro_offsets.y_axis + (1 << 2)) >> 3;
					sensors_data.gyro_offsets_SC.z_axis = (sensors_data.gyro_offsets.z_axis + (1 << 2)) >> 3;
					break;

				case (GYRO_FS_1000dps):
					sensors_data.gyro_offsets_SC.x_axis = (sensors_data.gyro_offsets.x_axis + (1 << 1)) >> 2;
					sensors_data.gyro_offsets_SC.y_axis = (sensors_data.gyro_offsets.y_axis + (1 << 1)) >> 2;
					sensors_data.gyro_offsets_SC.z_axis = (sensors_data.gyro_offsets.z_axis + (1 << 1)) >> 2;
					break;

				case (GYRO_FS_500dps):
					sensors_data.gyro_offsets_SC.x_axis = (sensors_data.gyro_offsets.x_axis + (1 << 0)) >> 1;
					sensors_data.gyro_offsets_SC.y_axis = (sensors_data.gyro_offsets.y_axis + (1 << 0)) >> 1;
					sensors_data.gyro_offsets_SC.z_axis = (sensors_data.gyro_offsets.z_axis + (1 << 0)) >> 1;
					break;

				case (GYRO_FS_250dps):
					sensors_data.gyro_offsets_SC.x_axis = sensors_data.gyro_offsets.x_axis;
					sensors_data.gyro_offsets_SC.y_axis = sensors_data.gyro_offsets.y_axis;
					sensors_data.gyro_offsets_SC.z_axis = sensors_data.gyro_offsets.z_axis;
					break;

				default :
					sensors_data.gyro_offsets_SC.x_axis = sensors_data.gyro_offsets.x_axis;
					sensors_data.gyro_offsets_SC.y_axis = sensors_data.gyro_offsets.y_axis;
					sensors_data.gyro_offsets_SC.z_axis = sensors_data.gyro_offsets.z_axis;
					break;
			}

			switch(sensors_params.acc_FS<<3)
			{
				case (ACC_FS_16g):
					sensors_data.acc_offsets_SC.x_axis = (sensors_data.acc_offsets.x_axis + (1 << 2)) >> 3;
					sensors_data.acc_offsets_SC.y_axis = (sensors_data.acc_offsets.y_axis + (1 << 2)) >> 3;
					sensors_data.acc_offsets_SC.z_axis = (sensors_data.acc_offsets.z_axis + (1 << 2)) >> 3;
					break;

				case (ACC_FS_8g):
					sensors_data.acc_offsets_SC.x_axis = (sensors_data.acc_offsets.x_axis + (1 << 1)) >> 2;
					sensors_data.acc_offsets_SC.y_axis = (sensors_data.acc_offsets.y_axis + (1 << 1)) >> 2;
					sensors_data.acc_offsets_SC.z_axis = (sensors_data.acc_offsets.z_axis + (1 << 1)) >> 2;
					break;

				case (ACC_FS_4g):
					sensors_data.acc_offsets_SC.x_axis = (sensors_data.acc_offsets.x_axis + (1 << 0)) >> 1;
					sensors_data.acc_offsets_SC.y_axis = (sensors_data.acc_offsets.y_axis + (1 << 0)) >> 1;
					sensors_data.acc_offsets_SC.z_axis = (sensors_data.acc_offsets.z_axis + (1 << 0)) >> 1;
					break;

				case (ACC_FS_2g):
					sensors_data.acc_offsets_SC.x_axis = sensors_data.acc_offsets.x_axis;
					sensors_data.acc_offsets_SC.y_axis = sensors_data.acc_offsets.y_axis;
					sensors_data.acc_offsets_SC.z_axis = sensors_data.acc_offsets.z_axis;
					break;

				default :
					sensors_data.acc_offsets_SC.x_axis = sensors_data.acc_offsets.x_axis;
					sensors_data.acc_offsets_SC.y_axis = sensors_data.acc_offsets.y_axis;
					sensors_data.acc_offsets_SC.z_axis = sensors_data.acc_offsets.z_axis;
					break;
			}
		}
                */
		//else if(sensors_params.ACC_mounted & LIS2DH12)
                if(sensors_params.ACC_mounted & LIS2DH12)
		{
		    uint8_t ODR;                                                                //accelerometer output data rate is based on TDOA refresh rate
		    if(tdoaParameters.refresh_rate_ms >= 1000) ODR = LIS2DH12_ODR1_Hz;
		    else if(tdoaParameters.refresh_rate_ms >= 100) ODR = LIS2DH12_ODR10_Hz;
		    else if(tdoaParameters.refresh_rate_ms >= 40) ODR = LIS2DH12_ODR25_Hz;
		    else if(tdoaParameters.refresh_rate_ms >= 20) ODR = LIS2DH12_ODR50_Hz;
		    else if(tdoaParameters.refresh_rate_ms >= 10) ODR = LIS2DH12_ODR100_Hz;
		    else ODR = LIS2DH12_ODR200_Hz;
		    lis2dh12_set_ODR(ODR);                                                      //set acc output data rate

			lis2dh12_set_FS(sensors_params.acc_FS << 1);                                //set full scale of accelerometer - that is possible only for raw data
			lis2dh12_set_all_axes_active();                                             //all accelerometer axes should be measured
			lis2dh12_set_precision(LIS2DH12_HR_MODE);                                   //set high resolution of acc measurement
			lis2dh12_set_Active();                                                      //switch acc to active mode
			switch(sensors_params.acc_FS)
			{
				case (ACC_FS_8g):
					sensors_data.acc_offsets_SC.x_axis = (sensors_data.acc_offsets.x_axis + (1 << 1)) >> 2;
					sensors_data.acc_offsets_SC.y_axis = (sensors_data.acc_offsets.y_axis + (1 << 1)) >> 2;
					sensors_data.acc_offsets_SC.z_axis = (sensors_data.acc_offsets.z_axis + (1 << 1)) >> 2;
					break;

				case (ACC_FS_4g):
					sensors_data.acc_offsets_SC.x_axis = (sensors_data.acc_offsets.x_axis + (1 << 0)) >> 1;
					sensors_data.acc_offsets_SC.y_axis = (sensors_data.acc_offsets.y_axis + (1 << 0)) >> 1;
					sensors_data.acc_offsets_SC.z_axis = (sensors_data.acc_offsets.z_axis + (1 << 0)) >> 1;
					break;

				case (ACC_FS_2g):
					sensors_data.acc_offsets_SC.x_axis = sensors_data.acc_offsets.x_axis;
					sensors_data.acc_offsets_SC.y_axis = sensors_data.acc_offsets.y_axis;
					sensors_data.acc_offsets_SC.z_axis = sensors_data.acc_offsets.z_axis;
					break;

				default :
					sensors_data.acc_offsets_SC.x_axis = sensors_data.acc_offsets.x_axis;
					sensors_data.acc_offsets_SC.y_axis = sensors_data.acc_offsets.y_axis;
					sensors_data.acc_offsets_SC.z_axis = sensors_data.acc_offsets.z_axis;
					break;
			}
		}
	}


/**
 * @brief This function send settings into sensors via I2C bus.
 */
void configure_sensors(void)
{
	//I2C_pins_enable();
	sensors_config();

}


void read_sensors_data(void)
{
    //sens_raw_data_read();	//read sensors raw data only if that is required
    lis2dh12_read_raw_acc_data(sensors_params.acc_FS << 1);
    printf("test\n\r");//khs
    printf("sensors_data.acc_data.x_axis = %d\n\r",sensors_data.acc_data.x_axis);
    printf("sensors_data.acc_data.y_axis = %d\n\r",sensors_data.acc_data.y_axis);
    printf("sensors_data.acc_data.z_axis = %d\n\r",sensors_data.acc_data.z_axis);
}

//------------------------------------------------------------------------------NFC
bool check_NFC_EEPROM(void)
{


    if(is_NFC_ok())
    {
       return false;
    }
    else
    {
//        signalize_EEPROM_error();//org_display
       // do_SWResetMCU(15);
       printf("NFC_FAIL....\n\r");
       return true;
    }

}