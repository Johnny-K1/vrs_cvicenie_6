/*
 * lps25hb.c
 *
 *  Created on: 6. 11. 2022
 *      Author: jkose
 */
#include "lps25hb.h"
#include "math.h"

#define roundz(x,d) ((floor(((x)*pow(10,d))+.5))/pow(10,d))

uint8_t read_addres_lps = LPS25HB_DEVICE_ADDRESS_READ_1;
uint8_t write_addres_lps = LPS25HB_DEVICE_ADDRESS_WRITE_1;

uint8_t lps25hb_read_byte(uint8_t reg_addr)
{
	uint8_t data = 0;
	return *(i2c_master_read(&data, 1, reg_addr, read_addres_lps, 0));
}


void lps25hb_write_byte(uint8_t reg_addr, uint8_t value)
{
	i2c_master_write(value, reg_addr, write_addres_lps, 0);
}

void lps25hb_readArray(uint8_t * data, uint8_t reg, uint8_t length)
{
	i2c_master_read(data, length, reg, read_addres_lps, 1);
}

void lps25hb_get_pressure(float *pressure){
	uint8_t buffer[3];
	int16_t press_out_H, press_out_L, press_out_XL;
	int32_t temp;
	float pressure_counts;

	lps25hb_readArray(buffer, LPS25HB_PRESS_OUT_XL, 3);

	press_out_XL = ((uint16_t)buffer[0]);
	press_out_L = ((uint16_t)buffer[1]);
	press_out_H = ((uint16_t)buffer[2]);

	temp = (press_out_H << 8) | press_out_L;

	pressure_counts = (temp << 8) | press_out_XL;

	*pressure = pressure_counts/4096;
	*pressure = roundz(*pressure,2);
}

void lps25hb_get_height(float* height, float pressure, float T, uint16_t RH, float reference_height){
	double p = pressure*100;
	double Rv = 461.495;
	double Rd = 287.058;

	double T1 = T + 273.15;
	double g = 9.81;

	double p1 = 6.1078 * pow(10,((7.5 * T)/(T+237.3)));
	double pv = p1 * RH;
	double pd = p - pv;

	double ro = (pd / (Rd * T1)) + (pv/(Rv*T1));

	*height = (pressure/(g*ro))-reference_height;
	*height = roundz(*height,2);
}

uint8_t lps25hb_init(void)
{

	uint8_t status = 1;

	//LIS3MDL_ACC_ON;

	LL_mDelay(100);

	uint8_t val = lps25hb_read_byte(LPS25HB_WHO_AM_I_ADDRES);

	if(val == LPS25HB_WHO_AM_I_VALUE)
	{
		status = 1;
	}
	else			//if the device is not found on one address, try another one
	{
		read_addres_lps = LPS25HB_DEVICE_ADDRESS_READ_2;
		write_addres_lps = LPS25HB_DEVICE_ADDRESS_WRITE_2;
		val = lps25hb_read_byte(LPS25HB_WHO_AM_I_ADDRES);
		if(val == LPS25HB_WHO_AM_I_VALUE)
		{
			status = 1;
		}
		else
		{
			status = 0;
			//return status;
		}
	}

	//acc device init

	uint8_t ctrl1 = 176; // +-2g res
	lps25hb_write_byte(LPS25HB_ADDRESS_CTRL1, ctrl1);


	return status;
}


