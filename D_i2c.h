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
#define D_I2C_DISABLED					0U
#define D_I2C_ENABLED					1U

	/* I2C_CR1 */
#define D_I2C_SW_RST_Pos				15U
#define D_I2C_SW_RST_EN					1U
#define D_I2C_SW_RST_DIS				0U

#define D_I2C_PEC_Pos					12U  /* Indicates whether the current or next byte in the shift reggister is a Packet Error Checking */
#define D_I2C_PEC_TRANS					1U
#define D_I2C_NO_PEC_TRANS				0U

/* POS??? */

#define D_I2C_ACK_Pos					10U
#define D_I2C_ACK_ACK					1U
#define D_I2C_ACK_NACK					0U

#define D_I2C_STOP_Pos					9U
#define D_I2C_STOP_NOSTOP				0U
#define D_I2C_STOP_STOP					1U  /* Master: Stop after current byte sent or after current START sent
											 Slave: Release SDA and SCL after current byte sent */

#define D_I2C_START_Pos					8U
#define D_I2C_START_START				1U
#define D_I2C_START_NOSTART				0U 	/* Master: Repeated START generation
											 Slave: START generation as soon as bus is free*/

#define D_I2C_NOSTRETCH_Pos				7U /* SLAVE mode only*/
#define D_I2C_NOSTRETCH_EN				1U
#define D_I2C_NOSTRETCH_DIS				0U

#define D_I2C_GENCALL_Pos				6U /* General call enabled*/
#define D_I2C_GENCALL_EN				1U /* Addr 00h is ACK-ed */
#define D_I2C_GENCALL_DIS				0U /* Addr 00h is NACK-ed */

#define D_I2C_PECEN_Pos					5U  /* Packet Error Checking Enable */
#define D_I2C_PECEN						1U
#define D_I2C_PECEN_NOT					0U

#define D_I2C_PERIPH_EN_Pos				0U
#define D_I2C_PERIPH_EN					1U
#define D_I2C_PERIPH_DISABLE			0U

/* I2C_CR2 */


/*Address mode*/
#define D_I2C_ADDMODE_Pos				15U
#define D_I2C_ADDMODE_7BIT				0x0000
#define D_I2C_ADDMODE_10BIT				(1U << D_I2C_ADDMODE_Pos)

#define D_I2C_OWNADDR7BIT_Pos			1U
#define D_I2C_OWNADDR10BIT_Pos			0U

/* DualAddressMode */
#define D_I2C_ENDUALADDR_Pos			0U
#define D_I2C_ENDUALADDR_EN				1U
#define D_I2C_ENDUALADDR_DISABLE		0U
#define D_I2C_OWNADDR2_Pos				1U

/* Speed */
/* NoStretchMode */
/* GeneralCallMode */

/* Function prototypes */
void D_i2c_init(I2C_TypeDef* I2c_Periph, I2C_InitTypeDef* InitStruct);

#endif /* I2C_DRIVER_D_I2C_H_ */
