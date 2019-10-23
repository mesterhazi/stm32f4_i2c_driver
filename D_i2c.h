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
	uint32_t AddressMode;		/* 7bit or 10bit addressing in I2C_OAR1 reg: D_I2C_ADDRMODE_7B/10B */
	uint32_t OwnAddress1;		/* For Single Address mode and for Dual Address mode */
	uint32_t DualAddressMode;	/* Dual addressing enable in I2C_OAR2 reg: D_I2C_ENDUALADDR_EN/DISABLE */
	uint32_t OwnAddress2;		/* Secondary Address for Dual Address mode */
	uint32_t ClockSpeed;				/* I2C clock speed up to 400kHz  */
	uint32_t DutyCycle;				/* Duty cycle in Fast mode: D_I2C_CCR_DUTY_16p9 or D_I2C_CCR_DUTY_2*/
	uint32_t NoStretchMode;		/* Clock stretching enable : D_I2C_NOSTRETCH_EN/DIS*/
	uint32_t GeneralCallMode;	/* General call enable : D_I2C_GENCALL_EN/DIS*/
}D_I2C_InitTypeDef;

/* Macros  */
#define D_I2C_DISABLED					0U
#define D_I2C_ENABLED					1U
#define D_I2C_ADDR_10BIT_HEAD			0x11110000  /*0x11110xx0*/

/* InitTypeDef value definitions */

/*Address mode*/
#define D_I2C_ADDMODE_Pos				15U
#define D_I2C_ADDMODE_7BIT				0x0000
#define D_I2C_ADDMODE_10BIT				(1U << D_I2C_ADDMODE_Pos)

#define D_I2C_OWNADDR7BIT_Pos			1U
#define D_I2C_OWNADDR10BIT_Pos			0U

	/* I2C_CR1 */
#define D_I2C_SW_RST_Pos				15U
#define D_I2C_SW_RST_EN					(1U << D_I2C_SW_RST_Pos)
#define D_I2C_SW_RST_DIS				0U

#define D_I2C_PEC_TRANS_Pos					12U  /* Indicates whether the current or next byte in the shift reggister is a Packet Error Checking */
#define D_I2C_PEC_TRANS					(1U << D_I2C_PEC_TRANS_Pos)
#define D_I2C_NO_PEC_TRANS				0U

/* POS??? */

#define D_I2C_ACK_Pos					10U
#define D_I2C_ACK_ACK					(1U << D_I2C_ACK_Pos)
#define D_I2C_ACK_NACK					0U

#define D_I2C_STOP_Pos					9U
#define D_I2C_STOP_NOSTOP				0U
#define D_I2C_STOP_STOP					(1U << D_I2C_STOP_Pos)  /* Master: Stop after current byte sent or after current START sent
											 Slave: Release SDA and SCL after current byte sent */

#define D_I2C_START_Pos					8U
#define D_I2C_START_START				(1U << D_I2C_START_Pos)
#define D_I2C_START_NOSTART				0U 	/* Master: Repeated START generation
											 Slave: START generation as soon as bus is free*/

#define D_I2C_NOSTRETCH_Pos				7U /* SLAVE mode only*/
#define D_I2C_NOSTRETCH_EN				(1U << D_I2C_NOSTRETCH_Pos)
#define D_I2C_NOSTRETCH_DIS				0U

#define D_I2C_GENCALL_Pos				6U /* General call enabled*/
#define D_I2C_GENCALL_EN				(1U << D_I2C_GENCALL_Pos) /* Addr 00h is ACK-ed */
#define D_I2C_GENCALL_DIS				0U /* Addr 00h is NACK-ed */

#define D_I2C_PECEN_Pos					5U  /* Packet Error Checking Enable */
#define D_I2C_PECEN						(1U << D_I2C_PECEN_Pos)
#define D_I2C_PECEN_NOT					0U

#define D_I2C_PERIPH_EN_Pos				0U
#define D_I2C_PERIPH_EN					1U
#define D_I2C_PERIPH_DISABLE			0U

/* I2C_CR2 */
#define D_I2C_CLKFREQ_Pos				0U /* Min value = 2U max value 50U 6bits [MHz]*/
#define D_I2C_CLKFREQ_Mask				0x003F

#define D_I2C_IT_ERR_Pos				8U /* Error interrupt enable */
#define D_I2C_IT_ERR_EN					(1U << D_I2C_IT_ERR_Pos)
#define D_I2C_IT_ERR_DISABLE			0U

#define D_I2C_IT_EVT_Pos				9U /* Event interrupt enable */
#define D_I2C_IT_EVT_EN					(1U << D_I2C_IT_EVT_Pos)
#define D_I2C_IT_EVT_DISABLE			0U

#define D_I2C_IT_BUF_Pos				10U /* Buffer interrupt enable */
#define D_I2C_IT_BUF_EN					(1U << D_I2C_IT_BUF_Pos)
#define D_I2C_IT_BUF_DISABLE			0U

#define D_I2C_DMA_REQ_Pos				11U
#define D_I2C_DMA_REQ_EN				(1U << D_I2C_DMA_REQ_Pos)
#define D_I2C_DMA_REQ_DISABLE			0U

#define D_I2C_DMA_LAST_Pos				12U /* This bit is used in master receiver mode to permit the generation of a NACK on the last received data. */
#define D_I2C_DMA_EOT					(1U << D_I2C_DMA_LAST_Pos)  /* Next DMA EOT is the last transfer */
#define D_I2C_DMA_NO_EOT				0U /* Next DMA EOT is the last transfer */



/* DualAddressMode */
#define D_I2C_ENDUALADDR_Pos			0U
#define D_I2C_ENDUALADDR_EN				1U
#define D_I2C_ENDUALADDR_DISABLE		0U
#define D_I2C_OWNADDR2_Pos				1U

/* I2C_DR */
#define D_I2C_DR_MASK					0x00FF

/* I2C_SR1 */
#define D_I2C_FLAG_START_Pos			0U  /* Start bit - Master mode */
#define D_I2C_FLAG_START_GENERATED		1U
#define D_I2C_FLAG_START_NOTGEN			0U

#define D_I2C_FLAG_ADDR_Pos				1U  /* Address sent - Master mode | Address matched - Slave mode */
#define D_I2C_FLAG_ADDR_ACK				(1U << D_I2C_FLAG_ADDR_Pos)
#define D_I2C_FLAG_ADDR_NACK			0U

#define D_I2C_FLAG_BYTE_FIN_Pos			2U /* Byte transfer finished */
#define D_I2C_FLAG_BYTE_FINISHED		(1U << D_I2C_FLAG_BYTE_FIN_Pos)
#define D_I2C_FLAG_BYTE_NOTFINISHED		0U

#define D_I2C_FLAG_ADD10_Pos			3U /* 10bit header sent - Master mode */
#define D_I2C_FLAG_ADD10_SENT			(1U << D_I2C_FLAG_ADD10_Pos) /* First byte sent from 10bit address */
#define D_I2C_FLAG_ADD10_NOEVT			0U

#define D_I2C_FLAG_STOPF_Pos			4U /* Stop detection - Slave mode */
#define D_I2C_FLAG_STOP_STOP			(1U << D_I2C_FLAG_STOPF_Pos)
#define D_I2C_FLAG_STOP_NOSTOP			0U

#define D_I2C_FLAG_RxNE_Pos				6U /* Data register not empty - receiver mode */
#define D_I2C_FLAG_RxNE_EMPTY			0U
#define D_I2C_FLAG_RxNE_NEMPTY			(1U << D_I2C_FLAG_RxNE_Pos)

#define D_I2C_FLAG_TxE_Pos				7U /* Date register not empty - transmitter mode */
#define D_I2C_FLAG_TxE_EMPTY			0U
#define D_I2C_FLAG_TxE_NEMPTY			(1U << D_I2C_FLAG_TxE_Pos)

#define D_I2C_FLAG_BERR_Pos				8U /* Bus error - misplaced START or STOP condition */
#define D_I2C_FLAG_BERR_NOERR			0U
#define D_I2C_FLAG_BERR_ERR				(1U << D_I2C_FLAG_BERR_Pos)

#define D_I2C_FLAG_ARLO_Pos				9U /* Arbitration lost - Master mode */
#define D_I2C_FLAG_ARLO_LOST			(1U << D_I2C_FLAG_ARLO_Pos)
#define D_I2C_FLAG_ARLO_NOTLOST			0U

#define D_I2C_FLAG_AF_Pos				10U /* ACK failure */
#define D_I2C_FLAG_AF_FAIL				(1U << D_I2C_FLAG_AF_Pos)
#define D_I2C_FLAG_AF_NOFAIL			0U

#define D_I2C_FLAG_OVR_Pos				11U /* Overrun/Underrun */
#define D_I2C_FLAG_OVR_PRESENT			(1U << D_I2C_FLAG_OVR_Pos)
#define D_I2C_FLAG_OVR_NOTPRESENT		0U

#define D_I2C_FLAG_PECERR_Pos			12U /* PEC error in reception */
#define D_I2C_FLAG_PECERR_ACK			0U
#define D_I2C_FLAG_PECERR_NACK			(1U << D_I2C_FLAG_PECERR_Pos)

#define D_I2C_FLAG_TIMEOUT_Pos			14U /* Timeout error */
#define D_I2C_FLAG_TIMEOUT_TOUT			(1U << D_I2C_FLAG_TIMEOUT_Pos) /* SCL low for 25ms */
#define D_I2C_FLAG_TIMEOUT_NOTOUT		0U

#define D_I2C_FLAG_SMBAL_Pos			15U /* SMBUS alert */
#define D_I2C_FLAG_SMBAL_ALERT			(1U << D_I2C_FLAG_SMBAL_Pos)
#define D_I2C_FLAG_SMBAL_NOALERT		0U

/* I2C_SR2 */
#define D_I2C_FLAG_MSL_Pos				0U
/* Master/Slave mode - Set by HW as soon as interface is Master mode
 * 		Reset by HW after Stop detected, Arbitration loss, or Periph disable*/
#define D_I2C_FLAG_MSL_MASTER			1U
#define D_I2C_FLAG_MSL_SLAVE			0U

#define D_I2C_FLAG_BUSY_Pos				1U /* Bus busy */
#define D_I2C_FLAG_BUSY_BUSY			(1U << D_I2C_FLAG_BUSY_Pos)
#define D_I2C_FLAG_BUSY_FREE			0U

#define D_I2C_FLAG_TRA_Pos				2U /* Transmitter/receiver set at the end of the address transfer*/
#define D_I2C_FLAG_TRA_SENT				(1U << D_I2C_FLAG_TRA_Pos)
#define D_I2C_FLAG_TRA_RECVD			0U

#define D_I2C_FLAG_GENCALL_Pos			4U /* General call address - Slave mode */
#define D_I2C_FLAG_GENCALL_RECVD		(1U << D_I2C_FLAG_GENCALL_Pos) /* General call address received */
#define D_I2C_FLAG_GENCALL_NOCALL		0U

#define D_I2C_FLAG_DUALF_Pos			7U /* Dual flag - Slave mode */
#define D_I2C_FLAG_DUALF_OAR1MATCH		0U /* Address matcher with OAR1 */
#define D_I2C_FLAG_DUALF_OAR2MATCH		(1U << D_I2C_FLAG_DUALF_Pos) /* Address matcher with OAR2 */

#define D_I2C_PEC_Pos					8U /* Packet error checking register */

/* Addresses */
#define D_I2C_OAR1_7B_Pos				1U
#define D_I2C_OAR1_10B_Pos				0U
#define D_I2C_OAR2_7B_Pos				1U

/* I2C_CCR */
#define D_I2C_CCR_MODE_Pos				15U
#define D_I2C_CCR_MODE_F				(1U << D_I2C_CCR_MODE_Pos)
#define D_I2C_CCR_MODE_S				0U

#define D_I2C_CCR_DUTY_Pos				14	/* FM mode t_low/t_high */
#define D_I2C_CCR_DUTY_16p9				(1U <<  D_I2C_CCR_DUTY_Pos) /* 16/9 */
#define D_I2C_CCR_DUTY_2				0U	/* 2 */

#define D_I2C_IS_SM(_speed_)			((_speed_) <= 100000)

#define D_I2C_CCR_FREQ_Pos				0
#define D_I2C_CCR_FREQ_MASK				0x0FFF  /* 12 bits information */

/* value checking */
#define D_I2C_IS_VALID_FREQ(_freq_)				((_freq_ < 50000000) && (_freq_ > 2000000)) /* between 2 and 50 MHz */


/* Function prototypes */
void D_i2c_init(I2C_TypeDef* I2C_Periph, D_I2C_InitTypeDef* InitStruct);
void D_i2c_Master_sendbytes(I2C_TypeDef *I2C_Periph, uint16_t pAddress, uint8_t *pTxData, uint8_t pIs10bitaddr);
void D_i2c_Master_readbytes(I2C_TypeDef *I2C_Periph, uint16_t pAddress, uint8_t *pRxBuf, uint16_t pSize, uint8_t pIs10bitaddr);

#endif /* I2C_DRIVER_D_I2C_H_ */





























