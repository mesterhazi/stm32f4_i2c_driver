/*
 * D_i2c.h
 *
 *  Created on: 2019. okt. 5.
 *      Author: adam
 */

#ifndef I2C_DRIVER_D_I2C_H_
#define I2C_DRIVER_D_I2C_H_

#include "stm32f407xx.h"

/* Typedefs */


typedef struct {
	uint32_t AddressMode;		// Switch between 7 or 10 bit addressing
	uint32_t OwnAddress1;		// For Single Address mode and for Dual Address mode
	uint32_t DualAddressMode;	// DualAddressMode enable
	uint32_t OwnAddress2;		// Secondary Address for Dual Address mode
	uint32_t Speed;				// Speed
	uint32_t NoStretchMode;		// Enable Clock stretching
	uint32_t GeneralCallMode;	// GeneralCallMode
	uint32_t DmaEnable;			// DmaEnable
}D_I2C_InitTypeDef;

/* Macros  */
#define D_I2C_DISABLED	 0U
#define D_I2C_ENABLED	 1U

/*Address mode*/

/* DualAddressMode */
/* Speed */
/* NoStretchMode */
/* GeneralCallMode */

/* Function prototypes */
void D_i2c_init(I2C_TypeDef* I2c_Periph, I2C_InitTypeDef* InitStruct);

#endif /* I2C_DRIVER_D_I2C_H_ */
