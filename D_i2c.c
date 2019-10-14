/*
 * D_i2c.c
 *
 *  Created on: 2019. okt. 5.
 *      Author: adam
 */

#include "D_i2c.h"


void D_i2c_init(I2C_TypeDef* I2C_Periph, I2C_InitTypeDef* InitStruct){
	/* Enable I2C peripherial */
	I2C_Periph->CR1 |= D_I2C_PERIPH_EN;

	I2C_Periph->OAR1 &= ~(1U << D_I2C_ADDMODE_Pos);
	I2C_Periph->OAR1 |= InitStruct->AddressingMode;

	I2C_Periph->OAR2 &= ~(1U << D_I2C_ENDUALADDR_Pos);
	I2C_Periph->OAR2 |= InitStruct->DualAddressMode;

	/* Own addresses */
	I2C_Periph->OAR1 &= ~0x03FF;  // clear 10bits of addr
	if(InitStruct->AddressingMode == D_I2C_ADDMODE_7BIT){
		I2C_Periph->OAR1 |= (InitStruct->OwnAddress1 & 0x7F) << D_I2C_OAR1_7B_Pos;

		if(InitStruct->DualAddressMode == D_I2C_ENDUALADDR_EN){
			I2C_Periph->OAR2 &= ~(0x3F << D_I2C_ENDUALADDR_Pos);
			I2C_Periph->OAR2 |= InitStruct->OwnAddress2 & 0x7F << D_I2C_OAR2_7B_Pos;
		}
	}

	I2C_Periph->CR2 &= ~D_I2C_CLKFREQ_Mask;
	I2C_Periph->CR2 |= InitStruct->ClockSpeed & D_I2C_CLKFREQ_Mask;

	/* Clock Control + Rise time reg */

	I2C_Periph->CR1 &= ~(1U << D_I2C_NOSTRETCH_Pos);
	I2C_Periph->CR1 |= InitStruct->NoStretchMode;

	I2C_Periph->CR1 &= ~(1U << D_I2C_GENCALL_Pos);
	I2C_Periph->CR1 |= InitStruct->GeneralCallMode;
}
