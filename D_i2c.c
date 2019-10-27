/*S
 * D_i2c.c
 *
 *  Created on: 2019. okt. 5.
 *      Author: adam
 */

#include "D_i2c.h"

uint16_t AHBP_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_Prescaler[4] = {2,4,8,16};
uint8_t PLL_P_values[4] = {2,4,6,8};

static void D_i2c_start(I2C_TypeDef* I2C_Periph);
static void D_i2c_stop(I2C_TypeDef* I2C_Periph);
static void D_i2c_ack_disable(I2C_TypeDef *I2C_Periph);
static void D_i2c_clear_ADDR(I2C_TypeDef *I2C_Periph);
static void D_i2c_sendaddr(I2C_TypeDef *I2C_Periph, uint16_t pAddress, uint8_t pRead_notwrite, uint8_t pIs10bitaddr);

uint32_t GetPllClock() {
	uint8_t  pllm, pllp;
	uint32_t pllvco, pllsource, pll_out;
	uint32_t HSE_VALUE=8000000, HSI_VALUE=16000000;

	pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
	pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

	if (pllsource != 0) {
		/* HSE used as PLL clock source */
		pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
	} else {
		/* HSI used as PLL clock source */
		pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
	}
	pllp = (RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16;
	pll_out = pllvco / pllp;

	return pll_out;
}


uint32_t Get_PCLK1(){
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0){
		SystemClk = 16000000; //HSI
	}else if(clksrc == 1){
		SystemClk = 8000000; // HSE
	}else {
		SystemClk = GetPllClock();
	}

	temp = (RCC->CFGR >> 4 ) & 0xF;
	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHBP_Prescaler[temp-8];
	}

	temp = (RCC->CFGR >> 10 ) & 0x7;
		if(temp < 4){
			apb1p = 1;
		}else{
			apb1p = APB1_Prescaler[temp-4];
		}

	pclk1 = (SystemClk / ahbp) / apb1p;
	return pclk1;

}
/* D_i2c_ClockEn
 * Enables or disables I2C peripherial clock
 * @param I2C_Periph: I2C peripherial pointer line I2C1
 * @param Enabled: D_I2C_DISABLED or D_I2C_ENABLED */
void D_i2c_ClockEn(I2C_TypeDef* I2C_Periph, uint8_t Enabled) {
	if (I2C_Periph == I2C1) {
		if (Enabled & 0x01) {
			RCC->APB1ENR |= D_I2C_CLOCK_EN_I2C1;
		} else {
			RCC->APB1ENR &= ~D_I2C_CLOCK_EN_I2C1;
		}
	} else if (I2C_Periph == I2C2) {
		if (Enabled & 0x01) {
			RCC->APB1ENR |= D_I2C_CLOCK_EN_I2C2;
		} else {
			RCC->APB1ENR &= ~D_I2C_CLOCK_EN_I2C2;
		}
	} else if (I2C_Periph == I2C3) {
		if (Enabled & 0x01) {
			RCC->APB1ENR |= D_I2C_CLOCK_EN_I2C3;
		} else {
			RCC->APB1ENR &= ~D_I2C_CLOCK_EN_I2C3;
		}
	}
}

void D_i2c_init(I2C_TypeDef* I2C_Periph, D_I2C_InitTypeDef* InitStruct){
	uint32_t pclk, ccr_val;
	pclk = Get_PCLK1(); /* Get peripherial clock speed */

	/* Disable I2C peripherial */
	I2C_Periph->CR1 &= ~D_I2C_PERIPH_EN;
	/* Set the CCR register BEFORE enabling the peripherial */
	I2C_Periph->CR2 &= ~0x3F;  /* clear lowest 5 bits*/
	I2C_Periph->CR2 |= ((uint8_t) (pclk / 1000000)) & 0x3F;

	/*  */
	if(D_I2C_IS_SM(InitStruct->ClockSpeed)){
		I2C_Periph->CCR &= ~(1U << D_I2C_CCR_MODE_Pos);  /* SM mode set */
		ccr_val = (uint32_t) pclk / (2 * InitStruct->ClockSpeed);

	}else{
		I2C_Periph->CCR |= (1U << D_I2C_CCR_MODE_Pos);  // FM mode set
		I2C_Periph->CCR &= ~(1U << D_I2C_CCR_DUTY_Pos);
		I2C_Periph->CCR |= InitStruct->DutyCycle;
		if(InitStruct->DutyCycle == D_I2C_CCR_DUTY_16p9){
			ccr_val = (uint32_t) pclk / (25 * InitStruct->ClockSpeed);
		}else {
			ccr_val = (uint32_t) pclk / (3 * InitStruct->ClockSpeed);
		}

	}
	I2C_Periph->CCR &= ~D_I2C_CCR_FREQ_MASK;
	I2C_Periph->CCR |= ccr_val & D_I2C_CCR_FREQ_MASK;


	/* Enable I2C peripherial */
	I2C_Periph->CR1 |= D_I2C_PERIPH_EN;

	I2C_Periph->OAR1 &= ~(1U << D_I2C_ADDMODE_Pos);
	I2C_Periph->OAR1 |= InitStruct->AddressMode;

	I2C_Periph->OAR2 &= ~(1U << D_I2C_ENDUALADDR_Pos);
	I2C_Periph->OAR2 |= InitStruct->DualAddressMode;

	/* Own addresses */
	I2C_Periph->OAR1 &= ~0x03FF;  // clear 10bits of addr
	if(InitStruct->AddressMode == D_I2C_ADDMODE_7BIT){
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

static void D_i2c_start(I2C_TypeDef *I2C_Periph){
	I2C_Periph->CR1 |= D_I2C_START_START;
	while(!(I2C_Periph->SR1 & D_I2C_FLAG_START_GENERATED)){}
}

static void D_i2c_stop(I2C_TypeDef *I2C_Periph){
	I2C_Periph->CR1 |= D_I2C_STOP_STOP;
}

static void D_i2c_ack_disable(I2C_TypeDef *I2C_Periph){
	I2C_Periph->CR1 &= ~(D_I2C_ACK_ACK);
}
static void D_i2c_clear_ADDR(I2C_TypeDef *I2C_Periph){
	uint32_t temp;
	temp = I2C_Periph->SR1;
	temp = I2C_Periph->SR1;
	(void)temp;
}

/* Send address on the selected I2C peripherial
 * This function sends a START signal then the address.
 * 7 bit and 10 bit addresses are supported
 * @param I2C_Periph:  I2C peripherial pointer
 * @param address:  I2C address if bits 8-10 are 0 7 bit address is expected
 * @param read_notwrite:  1:read request, 0:write request	*/
static void D_i2c_sendaddr(I2C_TypeDef *I2C_Periph, uint16_t pAddress, uint8_t pRead_notwrite, uint8_t pIs10bitaddr) {
	pRead_notwrite &= 0x01;  /* only 0x00 and 0x01 are expected */
	/* I2C START */
	D_i2c_start(I2C_Periph);
	/* Send Address */
	if (pIs10bitaddr & 0x01) {
		 /* header is 11110xx0 where xx is top 2 bits of addr */
		I2C_Periph->DR |= ((pAddress >> 7) & 0x6) | D_I2C_ADDR_10BIT_HEAD  | pRead_notwrite;
		while (I2C_Periph->SR1 & D_I2C_FLAG_ADD10_SENT) {
		} /* header sent */
		I2C_Periph->DR |= pAddress & 0xFF;
	} else {
		I2C_Periph->DR |= ((pAddress & 0x7F) << 1) | pRead_notwrite;	// 7 bit address + read_notwrite as LSB
	}
	while (I2C_Periph->SR1 & D_I2C_FLAG_ADDR_ACK) {
	} /* full address sent */

}

void D_i2c_Master_sendbytes(I2C_TypeDef *I2C_Periph, uint16_t pAddress, uint8_t *pTxData, uint8_t pIs10bitaddr){
	uint64_t i=0, len = 0;

	len = sizeof(pTxData)/sizeof(pTxData[0]);

	D_i2c_sendaddr(I2C_Periph, pAddress, 0x01, pIs10bitaddr);
	D_i2c_clear_ADDR(I2C_Periph);

	for(i=0; i<len; i++){
		while(I2C_Periph->SR1 & D_I2C_FLAG_TxE_EMPTY){}  /* Wait for empty shift register */
		I2C_Periph->DR |= pTxData[i];
	}
	/* wait for byte transfer finished */
	while(I2C_Periph->SR1 & D_I2C_FLAG_BYTE_FINISHED){}
	/* I2C STOP */
	I2C_Periph->CR1 |= D_I2C_STOP_STOP;
}

/* I2C master read
 * Starts an I2C transmission with writing an address
 * then receiving the answer from the slave device */
void D_i2c_Master_readbytes(I2C_TypeDef *I2C_Periph, uint16_t pAddress, uint8_t *pRxBuf, uint16_t pSize, uint8_t pIs10bitaddr){

	if(pIs10bitaddr & 0x01){
		D_i2c_sendaddr(I2C_Periph, pAddress, 0x00, pIs10bitaddr);
		D_i2c_clear_ADDR(I2C_Periph);
		/* Start repeat */
		D_i2c_start(I2C_Periph);
		/* after 10 bit addr sent the header shall be sent with LSB=1 */
		I2C_Periph->DR |= ((pAddress >> 7) & 0x6) | D_I2C_ADDR_10BIT_HEAD  | 0x01;
		/* disable ACK if only one byte is expected */
		if(pSize == 1){
			I2C_Periph->CR1 &= ~(D_I2C_ACK_ACK);
		}
		while (I2C_Periph->SR1 & D_I2C_FLAG_ADD10_SENT) {} /* header sent */
	} else{
		D_i2c_sendaddr(I2C_Periph, pAddress, 0x01, pIs10bitaddr);
		if(pSize == 1){  /* 1 byte read */
			D_i2c_ack_disable(I2C_Periph); /* Must disable ACK before clearing SR1 */
			while(!(I2C_Periph->SR1 & D_I2C_FLAG_ADDR_ACK)){}
			D_i2c_clear_ADDR(I2C_Periph);
			while(!(I2C_Periph->SR1 & D_I2C_FLAG_RxNE_NEMPTY)){}
			*pRxBuf = I2C_Periph->DR;
			D_i2c_stop(I2C_Periph);
			}
		else if(pSize == 2){
			D_i2c_ack_disable(I2C_Periph);
			/* POS=1 before data reception starts */
			I2C_Periph->CR1 |= (1U << D_I2C_POS_Pos);
			D_i2c_clear_ADDR(I2C_Periph);
			while(!(I2C_Periph->SR1 & D_I2C_FLAG_BYTE_FINISHED)){}
			/* Data1 in DR regm Data2 in shift reg */
			D_i2c_stop(I2C_Periph);
			*pRxBuf = I2C_Periph->DR;
			pRxBuf ++;
			*pRxBuf = I2C_Periph->DR;
		}
		else{
			D_i2c_clear_ADDR(I2C_Periph);
			while(pSize > 3){
				*pRxBuf = I2C_Periph->DR;
				pRxBuf ++;
				while(I2C_Periph->SR1 & D_I2C_FLAG_RxNE_NEMPTY){}
				pSize --;
			}
			while(!(I2C_Periph->SR1 & D_I2C_FLAG_BYTE_FINISHED)){}
			/* data N-2 is DR, N-1 in shift reg */
			D_i2c_ack_disable(I2C_Periph);
			*pRxBuf = I2C_Periph->DR;
			pRxBuf ++;
			while(!(I2C_Periph->SR1 & D_I2C_FLAG_BYTE_FINISHED)){}
			/* data N-1 is DR, N in shift reg */
			D_i2c_stop(I2C_Periph);
			*pRxBuf = I2C_Periph->DR;
			pRxBuf++;
			*pRxBuf = I2C_Periph->DR;
		}
	}
}

/* D_i2c_Master_writeregister
 * Writes a register identified by an offset (MAP)
 * I2C write to the given address, first data byte is the register selector (reg_addr), rest is the register value
 * @param pAddress: I2C address
 * @param reg_addr: register offset (MAP)
 * @param pTxBuf: register value array
 * @param pSize: size of the pTxBuf
 * @param pIs10bitaddr: flag 0:7bit, 1:10bit I2C address */
void D_i2c_Master_writeregister(I2C_TypeDef *I2C_Periph, uint16_t pAddress, uint8_t reg_addr, uint8_t *pTxBuf, uint16_t pSize, uint8_t pIs10bitaddr){
	uint8_t offset_plus_data[pSize+1];
	uint8_t index = 0;
	offset_plus_data[index] = reg_addr;
	index ++;
	while(pSize > 0){
		offset_plus_data[index] = *pTxBuf;
		index ++;
		pTxBuf ++;
		pSize --;
	}
	D_i2c_Master_sendbytes(I2C_Periph, pAddress, offset_plus_data, pIs10bitaddr);
}

/* D_i2c_Master_readregister
 * Reads a register identified by an offset (MAP)
 * Uses the read as defined for CS43L22 chip:
 * 1. I2c write address + MAP
 * 2. STOP
 * 3. I2c read
 * @param pAddress: I2C address
 * @param reg_addr: register offset (MAP)
 * @param pTxBuf: register value array
 * @param pSize: size of the pTxBuf
 * @param pIs10bitaddr: flag 0:7bit, 1:10bit I2C address */
void D_i2c_Master_readregister(I2C_TypeDef *I2C_Periph, uint16_t pAddress, uint8_t reg_addr, uint8_t *pRxBuf, uint16_t pSize, uint8_t pIs10bitaddr){
	D_i2c_Master_sendbytes(I2C_Periph, pAddress, &reg_addr, pIs10bitaddr);
	D_i2c_Master_readbytes(I2C_Periph, pAddress, pRxBuf, pSize, pIs10bitaddr);
}



























