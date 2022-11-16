/*
 * lps25hb.h
 *
 *  Created on: 6. 11. 2022
 *      Author: jkose
 */
#include "main.h"
#include "i2c.h"

#define 	LPS25HB_DEVICE_ADDRESS_READ_1			0xB9
#define 	LPS25HB_DEVICE_ADDRESS_WRITE_1			0xB8
#define 	LPS25HB_DEVICE_ADDRESS_READ_2			0xBB
#define 	LPS25HB_DEVICE_ADDRESS_WRITE_2			0xBA

#define 	LPS25HB_WHO_AM_I_VALUE					0xBD
#define 	LPS25HB_WHO_AM_I_ADDRES					0x0F

#define 	LPS25HB_ADDRESS_CTRL1					0x20
#define 	LPS25HB_ADDRESS_AV_CONF					0x10

#define 	LPS25HB_PRESS_OUT_XL					0x28
#define 	LPS25HB_PRESS_OUT_L						0x29
#define 	LPS25HB_PRESS_OUT_H						0x2A



uint8_t lps25hb_init(void);
uint8_t lps25hb_read_byte(uint8_t reg_addr);
void lps25hb_write_byte(uint8_t reg_addr, uint8_t value);

void lps25hb_get_pressure(float *pressure);
void lps25hb_get_height(float* height, float pressure, float T, uint16_t RH, float reference_height);
