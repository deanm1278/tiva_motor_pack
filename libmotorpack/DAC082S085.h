/*
 * DAC082S085.h
 *
 *  Created on: May 1, 2016
 *      Author: Dean Miller
 */

#ifndef DAC082S085_H_
#define DAC082S085_H_

#include <stdint.h>

#define DAC082S085_DAC0				0x01
#define DAC082S085_DAC1				0x05

#define DAC_SCLK_SPEED		18000000
#define DAC_BITS_PER_TXN	16

struct DAC082S085{
	uint32_t SSI_BASE;
	uint8_t AVAL;
	uint8_t BVAL;
};

void DAC082S085_init(struct DAC082S085 *dac, uint32_t SSI_BASE);
void DAC082S085_set_value(struct DAC082S085 *dac, uint8_t DAC_CHANNEL, uint8_t val);
uint8_t DAC082S085_get_value(struct DAC082S085 *dac, uint8_t DAC_CHANNEL);

#endif /* DAC082S085_H_ */
