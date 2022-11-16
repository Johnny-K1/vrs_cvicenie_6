/*
 * hts221.h
 *
 *  Created on: 2. 11. 2022
 *      Author: jkose
 */



#include "main.h"
#include "i2c.h"

#define 	HTS221_DEVICE_ADDRESS_READ				0xBF
#define 	HTS221_DEVICE_ADDRESS_WRITE				0xBE

#define 	HTS221_WHO_AM_I_VALUE					0xBC
#define 	HTS221_WHO_AM_I_ADDRES					0x0F

#define 	HTS221_ADDRESS_CTRL1					0x20
#define 	HTS221_ADDRESS_AV_CONF					0x10


#define 	HTS221_T0_DEGC_X8					  	0x32
#define 	HTS221_T0_T1_DEGC_H2					0x35
#define 	HTS221_T0_OUT_L							0x3C
#define 	HTS221_T1_OUT_L							0x3E
#define 	HTS221_TEMP_OUT_L_REG					0x2A

#define 	HTS221_H0_RH_X2							0x30
#define 	HTS221_H0_T0_OUT_L						0x36
#define 	HTS221_H1_T0_OUT_L						0x3A
#define 	HTS221_HR_OUT_L_REG						0x28


uint8_t hts221_init(void);
uint8_t hts221_read_byte(uint8_t reg_addr);
void hts221_write_byte(uint8_t reg_addr, uint8_t value);

void hts221_get_temp(float *temp);
void hts221_get_humidity(uint16_t* humidity);


