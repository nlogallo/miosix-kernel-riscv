/***************************************************************************
 *   Copyright (C) 2008, 2009, 2010, 2011, 2012, 2013, 2014                *
 *   by Terraneo Federico                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/ 

#ifndef SERIAL_GD32_H
#define SERIAL_GD32_H

#include "filesystem/console/console_device.h"
#include "kernel/sync.h"
#include "kernel/queue.h"
#include "interfaces/delays.h"

/* USARTx(x=0,1,2)/UARTx(x=3,4) definitions */
#define USART1                        USART_BASE                        /*!< USART1 base address */
#define USART2                        (USART_BASE+(0x00000400U))        /*!< USART2 base address */
#define UART3                         (USART_BASE+(0x00000800U))        /*!< UART3 base address */
#define UART4                         (USART_BASE+(0x00000C00U))        /*!< UART4 base address */
#define USART0                        (USART_BASE+(0x0000F400U))        /*!< USART0 base address */

/* registers definitions */
#define USART_STAT(usartx)            REG32((usartx) + (0x00000000U))   /*!< USART status register */
#define USART_DATA(usartx)            REG32((usartx) + (0x00000004U))   /*!< USART data register */
#define USART_BAUD(usartx)            REG32((usartx) + (0x00000008U))   /*!< USART baud rate register */
#define USART_CTL0(usartx)            REG32((usartx) + (0x0000000CU))   /*!< USART control register 0 */
#define USART_CTL1(usartx)            REG32((usartx) + (0x00000010U))   /*!< USART control register 1 */
#define USART_CTL2(usartx)            REG32((usartx) + (0x00000014U))   /*!< USART control register 2 */
#define USART_GP(usartx)              REG32((usartx) + (0x00000018U))   /*!< USART guard time and prescaler register */

/* bits definitions */
/* USARTx_STAT */
#define USART_STAT_PERR               BIT(0)                            /*!< parity error flag */
#define USART_STAT_FERR               BIT(1)                            /*!< frame error flag */
#define USART_STAT_NERR               BIT(2)                            /*!< noise error flag */
#define USART_STAT_ORERR              BIT(3)                            /*!< overrun error */
#define USART_STAT_IDLEF              BIT(4)                            /*!< IDLE frame detected flag */
#define USART_STAT_RBNE               BIT(5)                            /*!< read data buffer not empty */
#define USART_STAT_TC                 BIT(6)                            /*!< transmission complete */
#define USART_STAT_TBE                BIT(7)                            /*!< transmit data buffer empty */
#define USART_STAT_LBDF               BIT(8)                            /*!< LIN break detected flag */
#define USART_STAT_CTSF               BIT(9)                            /*!< CTS change flag */

/* USARTx_DATA */
#define USART_DATA_DATA               BITS(0,8)                         /*!< transmit or read data value */

/* USARTx_BAUD */
#define USART_BAUD_FRADIV             BITS(0,3)                         /*!< fraction part of baud-rate divider */
#define USART_BAUD_INTDIV             BITS(4,15)                        /*!< integer part of baud-rate divider */

/* USARTx_CTL0 */
#define USART_CTL0_SBKCMD             BIT(0)                            /*!< send break command */
#define USART_CTL0_RWU                BIT(1)                            /*!< receiver wakeup from mute mode */
#define USART_CTL0_REN                BIT(2)                            /*!< receiver enable */
#define USART_CTL0_TEN                BIT(3)                            /*!< transmitter enable */
#define USART_CTL0_IDLEIE             BIT(4)                            /*!< idle line detected interrupt enable */
#define USART_CTL0_RBNEIE             BIT(5)                            /*!< read data buffer not empty interrupt and overrun error interrupt enable */
#define USART_CTL0_TCIE               BIT(6)                            /*!< transmission complete interrupt enable */
#define USART_CTL0_TBEIE              BIT(7)                            /*!< transmitter buffer empty interrupt enable */
#define USART_CTL0_PERRIE             BIT(8)                            /*!< parity error interrupt enable */
#define USART_CTL0_PM                 BIT(9)                            /*!< parity mode */
#define USART_CTL0_PCEN               BIT(10)                           /*!< parity check function enable */
#define USART_CTL0_WM                 BIT(11)                           /*!< wakeup method in mute mode */
#define USART_CTL0_WL                 BIT(12)                           /*!< word length */
#define USART_CTL0_UEN                BIT(13)                           /*!< USART enable */

/* USARTx_CTL1 */
#define USART_CTL1_ADDR               BITS(0,3)                         /*!< address of USART */
#define USART_CTL1_LBLEN              BIT(5)                            /*!< LIN break frame length */
#define USART_CTL1_LBDIE              BIT(6)                            /*!< LIN break detected interrupt eanble */
#define USART_CTL1_CLEN               BIT(8)                            /*!< CK length */
#define USART_CTL1_CPH                BIT(9)                            /*!< CK phase */
#define USART_CTL1_CPL                BIT(10)                           /*!< CK polarity */
#define USART_CTL1_CKEN               BIT(11)                           /*!< CK pin enable */
#define USART_CTL1_STB                BITS(12,13)                       /*!< STOP bits length */
#define USART_CTL1_LMEN               BIT(14)                           /*!< LIN mode enable */

/* USARTx_CTL2 */
#define USART_CTL2_ERRIE              BIT(0)                            /*!< error interrupt enable */
#define USART_CTL2_IREN               BIT(1)                            /*!< IrDA mode enable */
#define USART_CTL2_IRLP               BIT(2)                            /*!< IrDA low-power */
#define USART_CTL2_HDEN               BIT(3)                            /*!< half-duplex enable */
#define USART_CTL2_NKEN               BIT(4)                            /*!< NACK enable in smartcard mode */
#define USART_CTL2_SCEN               BIT(5)                            /*!< smartcard mode enable */
#define USART_CTL2_DENR               BIT(6)                            /*!< DMA request enable for reception */
#define USART_CTL2_DENT               BIT(7)                            /*!< DMA request enable for transmission */
#define USART_CTL2_RTSEN              BIT(8)                            /*!< RTS enable */
#define USART_CTL2_CTSEN              BIT(9)                            /*!< CTS enable */
#define USART_CTL2_CTSIE              BIT(10)                           /*!< CTS interrupt enable */

/* USARTx_GP */
#define USART_GP_PSC                  BITS(0,7)                         /*!< prescaler value for dividing the system clock */
#define USART_GP_GUAT                 BITS(8,15)                        /*!< guard time value in smartcard mode */

/* constants definitions */
/* define the USART bit position and its register index offset */
#define USART_REGIDX_BIT(regidx, bitpos)     (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define USART_REG_VAL(usartx, offset)        (REG32((usartx) + (((uint32_t)(offset) & (0x0000FFFFU)) >> 6)))
#define USART_BIT_POS(val)                   ((uint32_t)(val) & (0x0000001FU))
#define USART_REGIDX_BIT2(regidx, bitpos, regidx2, bitpos2)   (((uint32_t)(regidx2) << 22) | (uint32_t)((bitpos2) << 16)\
                                                              | (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos)))
#define USART_REG_VAL2(usartx, offset)       (REG32((usartx) + ((uint32_t)(offset) >> 22)))
#define USART_BIT_POS2(val)                  (((uint32_t)(val) & (0x001F0000U)) >> 16)

/* register offset */
#define USART_STAT_REG_OFFSET                     (0x00000000U)        	/*!< STAT register offset */
#define USART_CTL0_REG_OFFSET                     (0x0000000CU)        	/*!< CTL0 register offset */
#define USART_CTL1_REG_OFFSET                     (0x00000010U)        	/*!< CTL1 register offset */
#define USART_CTL2_REG_OFFSET                     (0x00000014U)        	/*!< CTL2 register offset */


/* USART receiver configure */
#define CTL0_REN(regval)              (BIT(2) & ((uint32_t)(regval) << 2))
#define USART_RECEIVE_ENABLE          CTL0_REN(1)                       /*!< enable receiver */
#define USART_RECEIVE_DISABLE         CTL0_REN(0)                       /*!< disable receiver */

/* USART transmitter configure */
#define CTL0_TEN(regval)              (BIT(3) & ((uint32_t)(regval) << 3))
#define USART_TRANSMIT_ENABLE         CTL0_TEN(1)                       /*!< enable transmitter */
#define USART_TRANSMIT_DISABLE        CTL0_TEN(0)                       /*!< disable transmitter */

/* USART parity bits definitions */
#define CTL0_PM(regval)               (BITS(9,10) & ((uint32_t)(regval) << 9))
#define USART_PM_NONE                 CTL0_PM(0)                        /*!< no parity */
#define USART_PM_EVEN                 CTL0_PM(2)                        /*!< even parity */
#define USART_PM_ODD                  CTL0_PM(3)                        /*!< odd parity */

/* USART wakeup method in mute mode */
#define CTL0_WM(regval)               (BIT(11) & ((uint32_t)(regval) << 11))
#define USART_WM_IDLE                 CTL0_WM(0)                        /*!< idle line */
#define USART_WM_ADDR                 CTL0_WM(1)                        /*!< address match */

/* USART word length definitions */
#define CTL0_WL(regval)               (BIT(12) & ((uint32_t)(regval) << 12))
#define USART_WL_8BIT                 CTL0_WL(0)                        /*!< 8 bits */
#define USART_WL_9BIT                 CTL0_WL(1)                        /*!< 9 bits */

/* USART stop bits definitions */
#define CTL1_STB(regval)              (BITS(12,13) & ((uint32_t)(regval) << 12))
#define USART_STB_1BIT                CTL1_STB(0)                       /*!< 1 bit */
#define USART_STB_0_5BIT              CTL1_STB(1)                       /*!< 0.5 bit */
#define USART_STB_2BIT                CTL1_STB(2)                       /*!< 2 bits */
#define USART_STB_1_5BIT              CTL1_STB(3)                       /*!< 1.5 bits */

/* USART LIN break frame length */
#define CTL1_LBLEN(regval)            (BIT(5) & ((uint32_t)(regval) << 5))
#define USART_LBLEN_10B               CTL1_LBLEN(0)                     /*!< 10 bits */
#define USART_LBLEN_11B               CTL1_LBLEN(1)                     /*!< 11 bits */

/* USART CK length */
#define CTL1_CLEN(regval)             (BIT(8) & ((uint32_t)(regval) << 8))
#define USART_CLEN_NONE               CTL1_CLEN(0)                      /*!< there are 7 CK pulses for an 8 bit frame and 8 CK pulses for a 9 bit frame */
#define USART_CLEN_EN                 CTL1_CLEN(1)                      /*!< there are 8 CK pulses for an 8 bit frame and 9 CK pulses for a 9 bit frame */

/* USART clock phase */
#define CTL1_CPH(regval)              (BIT(9) & ((uint32_t)(regval) << 9))
#define USART_CPH_1CK                 CTL1_CPH(0)                       /*!< first clock transition is the first data capture edge */
#define USART_CPH_2CK                 CTL1_CPH(1)                       /*!< second clock transition is the first data capture edge */

/* USART clock polarity */
#define CTL1_CPL(regval)              (BIT(10) & ((uint32_t)(regval) << 10))
#define USART_CPL_LOW                 CTL1_CPL(0)                       /*!< steady low value on CK pin */
#define USART_CPL_HIGH                CTL1_CPL(1)                       /*!< steady high value on CK pin */

/* USART DMA request for receive configure */
#define CLT2_DENR(regval)             (BIT(6) & ((uint32_t)(regval) << 6))
#define USART_DENR_ENABLE             CLT2_DENR(1)                      /*!< DMA request enable for reception */
#define USART_DENR_DISABLE            CLT2_DENR(0)                      /*!< DMA request disable for reception */

/* USART DMA request for transmission configure */
#define USART_DENT_ENABLE             CLT2_DENT(1)                      /*!< DMA request enable for transmission */
#define USART_DENT_DISABLE            CLT2_DENT(0)                      /*!< DMA request disable for transmission */

/* USART RTS configure */
#define CLT2_RTSEN(regval)            (BIT(8) & ((uint32_t)(regval) << 8))
#define USART_RTS_ENABLE              CLT2_RTSEN(1)                     /*!< RTS enable */
#define USART_RTS_DISABLE             CLT2_RTSEN(0)                     /*!< RTS disable */

/* USART CTS configure */
#define CLT2_CTSEN(regval)            (BIT(9) & ((uint32_t)(regval) << 9))
#define USART_CTS_ENABLE              CLT2_CTSEN(1)                     /*!< CTS enable */
#define USART_CTS_DISABLE             CLT2_CTSEN(0)                     /*!< CTS disable */

/* USART IrDA low-power enable */
#define CTL2_IRLP(regval)             (BIT(2) & ((uint32_t)(regval) << 2))
#define USART_IRLP_LOW                CTL2_IRLP(1)                      /*!< low-power */
#define USART_IRLP_NORMAL             CTL2_IRLP(0)                      /*!< normal */

/* RCU definitions */
#define RCU                             RCU_BASE

/* registers definitions */

#define RCU_CTL                         REG32(RCU + 0x00U)        /*!< control register */
#define RCU_CFG0                        REG32(RCU + 0x04U)        /*!< clock configuration register 0 */
#define RCU_INT                         REG32(RCU + 0x08U)        /*!< clock interrupt register */
#define RCU_APB2RST                     REG32(RCU + 0x0CU)        /*!< APB2 reset register */
#define RCU_APB1RST                     REG32(RCU + 0x10U)        /*!< APB1 reset register */
#define RCU_AHBEN                       REG32(RCU + 0x14U)        /*!< AHB1 enable register */
#define RCU_APB2EN                      REG32(RCU + 0x18U)        /*!< APB2 enable register */
#define RCU_APB1EN                      REG32(RCU + 0x1CU)        /*!< APB1 enable register */
#define RCU_BDCTL                       REG32(RCU + 0x20U)        /*!< backup domain control register */
#define RCU_RSTSCK                      REG32(RCU + 0x24U)        /*!< reset source / clock register */
#define RCU_AHBRST                      REG32(RCU + 0x28U)        /*!< AHB reset register */
#define RCU_CFG1                        REG32(RCU + 0x2CU)        /*!< clock configuration register 1 */
#define RCU_DSV                         REG32(RCU + 0x34U)        /*!< deep-sleep mode voltage register */


/* bits definitions */
/* RCU_CTL */
#define RCU_CTL_IRC8MEN                 BIT(0)                    /*!< internal high speed oscillator enable */
#define RCU_CTL_IRC8MSTB                BIT(1)                    /*!< IRC8M high speed internal oscillator stabilization flag */
#define RCU_CTL_IRC8MADJ                BITS(3,7)                 /*!< high speed internal oscillator clock trim adjust value */
#define RCU_CTL_IRC8MCALIB              BITS(8,15)                /*!< high speed internal oscillator calibration value register */
#define RCU_CTL_HXTALEN                 BIT(16)                   /*!< external high speed oscillator enable */
#define RCU_CTL_HXTALSTB                BIT(17)                   /*!< external crystal oscillator clock stabilization flag */
#define RCU_CTL_HXTALBPS                BIT(18)                   /*!< external crystal oscillator clock bypass mode enable */
#define RCU_CTL_CKMEN                   BIT(19)                   /*!< HXTAL clock monitor enable */
#define RCU_CTL_PLLEN                   BIT(24)                   /*!< PLL enable */
#define RCU_CTL_PLLSTB                  BIT(25)                   /*!< PLL clock stabilization flag */
#define RCU_CTL_PLL1EN                  BIT(26)                   /*!< PLL1 enable */
#define RCU_CTL_PLL1STB                 BIT(27)                   /*!< PLL1 clock stabilization flag */
#define RCU_CTL_PLL2EN                  BIT(28)                   /*!< PLL2 enable */
#define RCU_CTL_PLL2STB                 BIT(29)                   /*!< PLL2 clock stabilization flag */


#define RCU_CFG0_SCS                    BITS(0,1)                 /*!< system clock switch */
#define RCU_CFG0_SCSS                   BITS(2,3)                 /*!< system clock switch status */
#define RCU_CFG0_AHBPSC                 BITS(4,7)                 /*!< AHB prescaler selection */
#define RCU_CFG0_APB1PSC                BITS(8,10)                /*!< APB1 prescaler selection */
#define RCU_CFG0_APB2PSC                BITS(11,13)               /*!< APB2 prescaler selection */
#define RCU_CFG0_ADCPSC                 BITS(14,15)               /*!< ADC prescaler selection */
#define RCU_CFG0_PLLSEL                 BIT(16)                   /*!< PLL clock source selection */
#define RCU_CFG0_PREDV0_LSB             BIT(17)                   /*!< the LSB of PREDV0 division factor */
#define RCU_CFG0_PLLMF                  BITS(18,21)               /*!< PLL clock multiplication factor */
#define RCU_CFG0_USBFSPSC               BITS(22,23)               /*!< USBFS clock prescaler selection */
#define RCU_CFG0_CKOUT0SEL              BITS(24,27)               /*!< CKOUT0 clock source selection */
#define RCU_CFG0_ADCPSC_2               BIT(28)                   /*!< bit 2 of ADCPSC */
#define RCU_CFG0_PLLMF_4                BIT(29)                   /*!< bit 4 of PLLMF */

/* RCU_INT */
#define RCU_INT_IRC40KSTBIF             BIT(0)                    /*!< IRC40K stabilization interrupt flag */
#define RCU_INT_LXTALSTBIF              BIT(1)                    /*!< LXTAL stabilization interrupt flag */
#define RCU_INT_IRC8MSTBIF              BIT(2)                    /*!< IRC8M stabilization interrupt flag */
#define RCU_INT_HXTALSTBIF              BIT(3)                    /*!< HXTAL stabilization interrupt flag */
#define RCU_INT_PLLSTBIF                BIT(4)                    /*!< PLL stabilization interrupt flag */
#define RCU_INT_PLL1STBIF               BIT(5)                    /*!< PLL1 stabilization interrupt flag */
#define RCU_INT_PLL2STBIF               BIT(6)                    /*!< PLL2 stabilization interrupt flag */
#define RCU_INT_CKMIF                   BIT(7)                    /*!< HXTAL clock stuck interrupt flag */
#define RCU_INT_IRC40KSTBIE             BIT(8)                    /*!< IRC40K stabilization interrupt enable */
#define RCU_INT_LXTALSTBIE              BIT(9)                    /*!< LXTAL stabilization interrupt enable */
#define RCU_INT_IRC8MSTBIE              BIT(10)                   /*!< IRC8M stabilization interrupt enable */
#define RCU_INT_HXTALSTBIE              BIT(11)                   /*!< HXTAL stabilization interrupt enable */
#define RCU_INT_PLLSTBIE                BIT(12)                   /*!< PLL stabilization interrupt enable */
#define RCU_INT_PLL1STBIE               BIT(13)                   /*!< PLL1 stabilization interrupt enable */
#define RCU_INT_PLL2STBIE               BIT(14)                   /*!< PLL2 stabilization interrupt enable */
#define RCU_INT_IRC40KSTBIC             BIT(16)                   /*!< IRC40K stabilization interrupt clear */
#define RCU_INT_LXTALSTBIC              BIT(17)                   /*!< LXTAL stabilization interrupt clear */
#define RCU_INT_IRC8MSTBIC              BIT(18)                   /*!< IRC8M stabilization interrupt clear */
#define RCU_INT_HXTALSTBIC              BIT(19)                   /*!< HXTAL stabilization interrupt clear */
#define RCU_INT_PLLSTBIC                BIT(20)                   /*!< PLL stabilization interrupt clear */
#define RCU_INT_PLL1STBIC               BIT(21)                   /*!< PLL1 stabilization interrupt clear */
#define RCU_INT_PLL2STBIC               BIT(22)                   /*!< PLL2 stabilization interrupt clear */
#define RCU_INT_CKMIC                   BIT(23)                   /*!< HXTAL clock stuck interrupt clear */

/* RCU_APB2RST */
#define RCU_APB2RST_AFRST               BIT(0)                    /*!< alternate function I/O reset */
#define RCU_APB2RST_PARST               BIT(2)                    /*!< GPIO port A reset */
#define RCU_APB2RST_PBRST               BIT(3)                    /*!< GPIO port B reset */
#define RCU_APB2RST_PCRST               BIT(4)                    /*!< GPIO port C reset */
#define RCU_APB2RST_PDRST               BIT(5)                    /*!< GPIO port D reset */
#define RCU_APB2RST_PERST               BIT(6)                    /*!< GPIO port E reset */
#define RCU_APB2RST_ADC0RST             BIT(9)                    /*!< ADC0 reset */
#define RCU_APB2RST_ADC1RST             BIT(10)                   /*!< ADC1 reset */
#define RCU_APB2RST_TIMER0RST           BIT(11)                   /*!< TIMER0 reset */
#define RCU_APB2RST_SPI0RST             BIT(12)                   /*!< SPI0 reset */
#define RCU_APB2RST_USART0RST           BIT(14)                   /*!< USART0 reset */

/* RCU_APB1RST */
#define RCU_APB1RST_TIMER1RST           BIT(0)                    /*!< TIMER1 reset */
#define RCU_APB1RST_TIMER2RST           BIT(1)                    /*!< TIMER2 reset */
#define RCU_APB1RST_TIMER3RST           BIT(2)                    /*!< TIMER3 reset */
#define RCU_APB1RST_TIMER4RST           BIT(3)                    /*!< TIMER4 reset */
#define RCU_APB1RST_TIMER5RST           BIT(4)                    /*!< TIMER5 reset */
#define RCU_APB1RST_TIMER6RST           BIT(5)                    /*!< TIMER6 reset */

#define RCU_APB1RST_WWDGTRST            BIT(11)                   /*!< WWDGT reset */
#define RCU_APB1RST_SPI1RST             BIT(14)                   /*!< SPI1 reset */
#define RCU_APB1RST_SPI2RST             BIT(15)                   /*!< SPI2 reset */
#define RCU_APB1RST_USART1RST           BIT(17)                   /*!< USART1 reset */
#define RCU_APB1RST_USART2RST           BIT(18)                   /*!< USART2 reset */
#define RCU_APB1RST_UART3RST            BIT(19)                   /*!< UART3 reset */
#define RCU_APB1RST_UART4RST            BIT(20)                   /*!< UART4 reset */
#define RCU_APB1RST_I2C0RST             BIT(21)                   /*!< I2C0 reset */
#define RCU_APB1RST_I2C1RST             BIT(22)                   /*!< I2C1 reset */
#define RCU_APB1RST_CAN0RST             BIT(25)                   /*!< CAN0 reset */
#define RCU_APB1RST_CAN1RST             BIT(26)                   /*!< CAN1 reset */
#define RCU_APB1RST_BKPIRST             BIT(27)                   /*!< backup interface reset */
#define RCU_APB1RST_PMURST              BIT(28)                   /*!< PMU reset */
#define RCU_APB1RST_DACRST              BIT(29)                   /*!< DAC reset */

/* RCU_AHBEN */
#define RCU_AHBEN_DMA0EN                BIT(0)                    /*!< DMA0 clock enable */
#define RCU_AHBEN_DMA1EN                BIT(1)                    /*!< DMA1 clock enable */
#define RCU_AHBEN_SRAMSPEN              BIT(2)                    /*!< SRAM clock enable when sleep mode */
#define RCU_AHBEN_FMCSPEN               BIT(4)                    /*!< FMC clock enable when sleep mode */
#define RCU_AHBEN_CRCEN                 BIT(6)                    /*!< CRC clock enable */
#define RCU_AHBEN_EXMCEN                BIT(8)                    /*!< EXMC clock enable */
#define RCU_AHBEN_USBFSEN               BIT(12)                   /*!< USBFS clock enable */

/* RCU_APB2EN */
#define RCU_APB2EN_AFEN                 BIT(0)                    /*!< alternate function IO clock enable */
#define RCU_APB2EN_PAEN                 BIT(2)                    /*!< GPIO port A clock enable */
#define RCU_APB2EN_PBEN                 BIT(3)                    /*!< GPIO port B clock enable */
#define RCU_APB2EN_PCEN                 BIT(4)                    /*!< GPIO port C clock enable */
#define RCU_APB2EN_PDEN                 BIT(5)                    /*!< GPIO port D clock enable */
#define RCU_APB2EN_PEEN                 BIT(6)                    /*!< GPIO port E clock enable */
#define RCU_APB2EN_ADC0EN               BIT(9)                    /*!< ADC0 clock enable */
#define RCU_APB2EN_ADC1EN               BIT(10)                   /*!< ADC1 clock enable */
#define RCU_APB2EN_TIMER0EN             BIT(11)                   /*!< TIMER0 clock enable */
#define RCU_APB2EN_SPI0EN               BIT(12)                   /*!< SPI0 clock enable */
#define RCU_APB2EN_USART0EN             BIT(14)                   /*!< USART0 clock enable */

/* RCU_APB1EN */
#define RCU_APB1EN_TIMER1EN             BIT(0)                    /*!< TIMER1 clock enable */
#define RCU_APB1EN_TIMER2EN             BIT(1)                    /*!< TIMER2 clock enable */
#define RCU_APB1EN_TIMER3EN             BIT(2)                    /*!< TIMER3 clock enable */
#define RCU_APB1EN_TIMER4EN             BIT(3)                    /*!< TIMER4 clock enable */
#define RCU_APB1EN_TIMER5EN             BIT(4)                    /*!< TIMER5 clock enable */
#define RCU_APB1EN_TIMER6EN             BIT(5)                    /*!< TIMER6 clock enable */
#define RCU_APB1EN_WWDGTEN              BIT(11)                   /*!< WWDGT clock enable */
#define RCU_APB1EN_SPI1EN               BIT(14)                   /*!< SPI1 clock enable */
#define RCU_APB1EN_SPI2EN               BIT(15)                   /*!< SPI2 clock enable */
#define RCU_APB1EN_USART1EN             BIT(17)                   /*!< USART1 clock enable */
#define RCU_APB1EN_USART2EN             BIT(18)                   /*!< USART2 clock enable */
#define RCU_APB1EN_UART3EN              BIT(19)                   /*!< UART3 clock enable */
#define RCU_APB1EN_UART4EN              BIT(20)                   /*!< UART4 clock enable */
#define RCU_APB1EN_I2C0EN               BIT(21)                   /*!< I2C0 clock enable */
#define RCU_APB1EN_I2C1EN               BIT(22)                   /*!< I2C1 clock enable */
#define RCU_APB1EN_CAN0EN               BIT(25)                   /*!< CAN0 clock enable */
#define RCU_APB1EN_CAN1EN               BIT(26)                   /*!< CAN1 clock enable */
#define RCU_APB1EN_BKPIEN               BIT(27)                   /*!< backup interface clock enable */
#define RCU_APB1EN_PMUEN                BIT(28)                   /*!< PMU clock enable */
#define RCU_APB1EN_DACEN                BIT(29)                   /*!< DAC clock enable */

/* RCU_BDCTL */
#define RCU_BDCTL_LXTALEN               BIT(0)                    /*!< LXTAL enable */
#define RCU_BDCTL_LXTALSTB              BIT(1)                    /*!< low speed crystal oscillator stabilization flag */
#define RCU_BDCTL_LXTALBPS              BIT(2)                    /*!< LXTAL bypass mode enable */
#define RCU_BDCTL_RTCSRC                BITS(8,9)                 /*!< RTC clock entry selection */
#define RCU_BDCTL_RTCEN                 BIT(15)                   /*!< RTC clock enable */
#define RCU_BDCTL_BKPRST                BIT(16)                   /*!< backup domain reset */

/* RCU_RSTSCK */
#define RCU_RSTSCK_IRC40KEN             BIT(0)                    /*!< IRC40K enable */
#define RCU_RSTSCK_IRC40KSTB            BIT(1)                    /*!< IRC40K stabilization flag */
#define RCU_RSTSCK_RSTFC                BIT(24)                   /*!< reset flag clear */
#define RCU_RSTSCK_EPRSTF               BIT(26)                   /*!< external pin reset flag */
#define RCU_RSTSCK_PORRSTF              BIT(27)                   /*!< power reset flag */
#define RCU_RSTSCK_SWRSTF               BIT(28)                   /*!< software reset flag */
#define RCU_RSTSCK_FWDGTRSTF            BIT(29)                   /*!< free watchdog timer reset flag */
#define RCU_RSTSCK_WWDGTRSTF            BIT(30)                   /*!< window watchdog timer reset flag */
#define RCU_RSTSCK_LPRSTF               BIT(31)                   /*!< low-power reset flag */

/* RCU_AHBRST */
#define RCU_AHBRST_USBFSRST             BIT(12)                   /*!< USBFS reset */

/* RCU_CFG1 */
#define RCU_CFG1_PREDV0                 BITS(0,3)                 /*!< PREDV0 division factor */
#define RCU_CFG1_PREDV1                 BITS(4,7)                 /*!< PREDV1 division factor */
#define RCU_CFG1_PLL1MF                 BITS(8,11)                /*!< PLL1 clock multiplication factor */
#define RCU_CFG1_PLL2MF                 BITS(12,15)               /*!< PLL2 clock multiplication factor */
#define RCU_CFG1_PREDV0SEL              BIT(16)                   /*!< PREDV0 input clock source selection */
#define RCU_CFG1_I2S1SEL                BIT(17)                   /*!< I2S1 clock source selection */
#define RCU_CFG1_I2S2SEL                BIT(18)                   /*!< I2S2 clock source selection  */

/* RCU_DSV */
#define RCU_DSV_DSLPVS                  BITS(0,1)                 /*!< deep-sleep mode voltage select */

/* constants definitions */
/* define the peripheral clock enable bit position and its register index offset */
#define RCU_REGIDX_BIT(regidx, bitpos)      (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define RCU_REG_VAL(periph)                 (REG32(RCU + ((uint32_t)(periph) >> 6)))
#define RCU_BIT_POS(val)                    ((uint32_t)(val) & 0x1FU)

/* register offset */
/* peripherals enable */
#define AHBEN_REG_OFFSET                0x14U                     /*!< AHB enable register offset */
#define APB1EN_REG_OFFSET               0x1CU                     /*!< APB1 enable register offset */
#define APB2EN_REG_OFFSET               0x18U                     /*!< APB2 enable register offset */

/* peripherals reset */
#define AHBRST_REG_OFFSET               0x28U                     /*!< AHB reset register offset */
#define APB1RST_REG_OFFSET              0x10U                     /*!< APB1 reset register offset */
#define APB2RST_REG_OFFSET              0x0CU                     /*!< APB2 reset register offset */
#define RSTSCK_REG_OFFSET               0x24U                     /*!< reset source/clock register offset */

/* clock control */
#define CTL_REG_OFFSET                  0x00U                     /*!< control register offset */
#define BDCTL_REG_OFFSET                0x20U                     /*!< backup domain control register offset */

/* clock stabilization and stuck interrupt */
#define INT_REG_OFFSET                  0x08U                     /*!< clock interrupt register offset */

/* configuration register */
#define CFG0_REG_OFFSET                 0x04U                     /*!< clock configuration register 0 offset */
#define CFG1_REG_OFFSET                 0x2CU                     /*!< clock configuration register 1 offset */

/* RCU_CFG0 register bit define */
/* system clock source select */
#define CFG0_SCS(regval)                (BITS(0,1) & ((uint32_t)(regval) << 0))
#define RCU_CKSYSSRC_IRC8M              CFG0_SCS(0)                         /*!< system clock source select IRC8M */
#define RCU_CKSYSSRC_HXTAL              CFG0_SCS(1)                         /*!< system clock source select HXTAL */
#define RCU_CKSYSSRC_PLL                CFG0_SCS(2)                         /*!< system clock source select PLL */

/* system clock source select status */
#define CFG0_SCSS(regval)               (BITS(2,3) & ((uint32_t)(regval) << 2))
#define RCU_SCSS_IRC8M                  CFG0_SCSS(0)                        /*!< system clock source select IRC8M */
#define RCU_SCSS_HXTAL                  CFG0_SCSS(1)                        /*!< system clock source select HXTAL */
#define RCU_SCSS_PLL                    CFG0_SCSS(2)                        /*!< system clock source select PLLP */

/* AHB prescaler selection */
#define CFG0_AHBPSC(regval)             (BITS(4,7) & ((uint32_t)(regval) << 4))
#define RCU_AHB_CKSYS_DIV1              CFG0_AHBPSC(0)                      /*!< AHB prescaler select CK_SYS */
#define RCU_AHB_CKSYS_DIV2              CFG0_AHBPSC(8)                      /*!< AHB prescaler select CK_SYS/2 */
#define RCU_AHB_CKSYS_DIV4              CFG0_AHBPSC(9)                      /*!< AHB prescaler select CK_SYS/4 */
#define RCU_AHB_CKSYS_DIV8              CFG0_AHBPSC(10)                     /*!< AHB prescaler select CK_SYS/8 */
#define RCU_AHB_CKSYS_DIV16             CFG0_AHBPSC(11)                     /*!< AHB prescaler select CK_SYS/16 */
#define RCU_AHB_CKSYS_DIV64             CFG0_AHBPSC(12)                     /*!< AHB prescaler select CK_SYS/64 */
#define RCU_AHB_CKSYS_DIV128            CFG0_AHBPSC(13)                     /*!< AHB prescaler select CK_SYS/128 */
#define RCU_AHB_CKSYS_DIV256            CFG0_AHBPSC(14)                     /*!< AHB prescaler select CK_SYS/256 */
#define RCU_AHB_CKSYS_DIV512            CFG0_AHBPSC(15)                     /*!< AHB prescaler select CK_SYS/512 */

/* APB1 prescaler selection */
#define CFG0_APB1PSC(regval)            (BITS(8,10) & ((uint32_t)(regval) << 8))
#define RCU_APB1_CKAHB_DIV1             CFG0_APB1PSC(0)                     /*!< APB1 prescaler select CK_AHB */
#define RCU_APB1_CKAHB_DIV2             CFG0_APB1PSC(4)                     /*!< APB1 prescaler select CK_AHB/2 */
#define RCU_APB1_CKAHB_DIV4             CFG0_APB1PSC(5)                     /*!< APB1 prescaler select CK_AHB/4 */
#define RCU_APB1_CKAHB_DIV8             CFG0_APB1PSC(6)                     /*!< APB1 prescaler select CK_AHB/8 */
#define RCU_APB1_CKAHB_DIV16            CFG0_APB1PSC(7)                     /*!< APB1 prescaler select CK_AHB/16 */

/* APB2 prescaler selection */
#define CFG0_APB2PSC(regval)            (BITS(11,13) & ((uint32_t)(regval) << 11))
#define RCU_APB2_CKAHB_DIV1             CFG0_APB2PSC(0)                     /*!< APB2 prescaler select CK_AHB */
#define RCU_APB2_CKAHB_DIV2             CFG0_APB2PSC(4)                     /*!< APB2 prescaler select CK_AHB/2 */
#define RCU_APB2_CKAHB_DIV4             CFG0_APB2PSC(5)                     /*!< APB2 prescaler select CK_AHB/4 */
#define RCU_APB2_CKAHB_DIV8             CFG0_APB2PSC(6)                     /*!< APB2 prescaler select CK_AHB/8 */
#define RCU_APB2_CKAHB_DIV16            CFG0_APB2PSC(7)                     /*!< APB2 prescaler select CK_AHB/16 */

/* ADC prescaler select */
#define RCU_CKADC_CKAPB2_DIV2           ((uint32_t)0x00000000U)             /*!< ADC prescaler select CK_APB2/2 */
#define RCU_CKADC_CKAPB2_DIV4           ((uint32_t)0x00000001U)             /*!< ADC prescaler select CK_APB2/4 */
#define RCU_CKADC_CKAPB2_DIV6           ((uint32_t)0x00000002U)             /*!< ADC prescaler select CK_APB2/6 */
#define RCU_CKADC_CKAPB2_DIV8           ((uint32_t)0x00000003U)             /*!< ADC prescaler select CK_APB2/8 */
#define RCU_CKADC_CKAPB2_DIV12          ((uint32_t)0x00000005U)             /*!< ADC prescaler select CK_APB2/12 */
#define RCU_CKADC_CKAPB2_DIV16          ((uint32_t)0x00000007U)             /*!< ADC prescaler select CK_APB2/16 */

/* PLL clock source selection */
#define RCU_PLLSRC_IRC8M_DIV2           ((uint32_t)0x00000000U)             /*!< IRC8M/2 clock selected as source clock of PLL */
#define RCU_PLLSRC_HXTAL                RCU_CFG0_PLLSEL                     /*!< HXTAL clock selected as source clock of PLL */

/* PLL clock multiplication factor */
#define PLLMF_4                         RCU_CFG0_PLLMF_4                    /* bit 4 of PLLMF */

#define CFG0_PLLMF(regval)              (BITS(18,21) & ((uint32_t)(regval) << 18))
#define RCU_PLL_MUL2                    CFG0_PLLMF(0)                       /*!< PLL source clock multiply by 2 */
#define RCU_PLL_MUL3                    CFG0_PLLMF(1)                       /*!< PLL source clock multiply by 3 */
#define RCU_PLL_MUL4                    CFG0_PLLMF(2)                       /*!< PLL source clock multiply by 4 */
#define RCU_PLL_MUL5                    CFG0_PLLMF(3)                       /*!< PLL source clock multiply by 5 */
#define RCU_PLL_MUL6                    CFG0_PLLMF(4)                       /*!< PLL source clock multiply by 6 */
#define RCU_PLL_MUL7                    CFG0_PLLMF(5)                       /*!< PLL source clock multiply by 7 */
#define RCU_PLL_MUL8                    CFG0_PLLMF(6)                       /*!< PLL source clock multiply by 8 */
#define RCU_PLL_MUL9                    CFG0_PLLMF(7)                       /*!< PLL source clock multiply by 9 */
#define RCU_PLL_MUL10                   CFG0_PLLMF(8)                       /*!< PLL source clock multiply by 10 */
#define RCU_PLL_MUL11                   CFG0_PLLMF(9)                       /*!< PLL source clock multiply by 11 */
#define RCU_PLL_MUL12                   CFG0_PLLMF(10)                      /*!< PLL source clock multiply by 12 */
#define RCU_PLL_MUL13                   CFG0_PLLMF(11)                      /*!< PLL source clock multiply by 13 */
#define RCU_PLL_MUL14                   CFG0_PLLMF(12)                      /*!< PLL source clock multiply by 14 */
#define RCU_PLL_MUL6_5                  CFG0_PLLMF(13)                      /*!< PLL source clock multiply by 6.5 */
#define RCU_PLL_MUL16                   CFG0_PLLMF(14)                      /*!< PLL source clock multiply by 16 */
#define RCU_PLL_MUL17                   (PLLMF_4 | CFG0_PLLMF(0))           /*!< PLL source clock multiply by 17 */
#define RCU_PLL_MUL18                   (PLLMF_4 | CFG0_PLLMF(1))           /*!< PLL source clock multiply by 18 */
#define RCU_PLL_MUL19                   (PLLMF_4 | CFG0_PLLMF(2))           /*!< PLL source clock multiply by 19 */
#define RCU_PLL_MUL20                   (PLLMF_4 | CFG0_PLLMF(3))           /*!< PLL source clock multiply by 20 */
#define RCU_PLL_MUL21                   (PLLMF_4 | CFG0_PLLMF(4))           /*!< PLL source clock multiply by 21 */
#define RCU_PLL_MUL22                   (PLLMF_4 | CFG0_PLLMF(5))           /*!< PLL source clock multiply by 22 */
#define RCU_PLL_MUL23                   (PLLMF_4 | CFG0_PLLMF(6))           /*!< PLL source clock multiply by 23 */
#define RCU_PLL_MUL24                   (PLLMF_4 | CFG0_PLLMF(7))           /*!< PLL source clock multiply by 24 */
#define RCU_PLL_MUL25                   (PLLMF_4 | CFG0_PLLMF(8))           /*!< PLL source clock multiply by 25 */
#define RCU_PLL_MUL26                   (PLLMF_4 | CFG0_PLLMF(9))           /*!< PLL source clock multiply by 26 */
#define RCU_PLL_MUL27                   (PLLMF_4 | CFG0_PLLMF(10))          /*!< PLL source clock multiply by 27 */
#define RCU_PLL_MUL28                   (PLLMF_4 | CFG0_PLLMF(11))          /*!< PLL source clock multiply by 28 */
#define RCU_PLL_MUL29                   (PLLMF_4 | CFG0_PLLMF(12))          /*!< PLL source clock multiply by 29 */
#define RCU_PLL_MUL30                   (PLLMF_4 | CFG0_PLLMF(13))          /*!< PLL source clock multiply by 30 */
#define RCU_PLL_MUL31                   (PLLMF_4 | CFG0_PLLMF(14))          /*!< PLL source clock multiply by 31 */
#define RCU_PLL_MUL32                   (PLLMF_4 | CFG0_PLLMF(15))          /*!< PLL source clock multiply by 32 */

/* USBFS prescaler select */
#define CFG0_USBPSC(regval)             (BITS(22,23) & ((uint32_t)(regval) << 22))
#define RCU_CKUSB_CKPLL_DIV1_5          CFG0_USBPSC(0)                      /*!< USBFS prescaler select CK_PLL/1.5 */
#define RCU_CKUSB_CKPLL_DIV1            CFG0_USBPSC(1)                      /*!< USBFS prescaler select CK_PLL/1 */
#define RCU_CKUSB_CKPLL_DIV2_5          CFG0_USBPSC(2)                      /*!< USBFS prescaler select CK_PLL/2.5 */
#define RCU_CKUSB_CKPLL_DIV2            CFG0_USBPSC(3)                      /*!< USBFS prescaler select CK_PLL/2 */

/* CKOUT0 clock source selection */
#define CFG0_CKOUT0SEL(regval)          (BITS(24,27) & ((uint32_t)(regval) << 24))
#define RCU_CKOUT0SRC_NONE              CFG0_CKOUT0SEL(0)                   /*!< no clock selected */
#define RCU_CKOUT0SRC_CKSYS             CFG0_CKOUT0SEL(4)                   /*!< system clock selected */
#define RCU_CKOUT0SRC_IRC8M             CFG0_CKOUT0SEL(5)                   /*!< internal 8M RC oscillator clock selected */
#define RCU_CKOUT0SRC_HXTAL             CFG0_CKOUT0SEL(6)                   /*!< high speed crystal oscillator clock (HXTAL) selected */
#define RCU_CKOUT0SRC_CKPLL_DIV2        CFG0_CKOUT0SEL(7)                   /*!< CK_PLL/2 clock selected */
#define RCU_CKOUT0SRC_CKPLL1            CFG0_CKOUT0SEL(8)                   /*!< CK_PLL1 clock selected */
#define RCU_CKOUT0SRC_CKPLL2_DIV2       CFG0_CKOUT0SEL(9)                   /*!< CK_PLL2/2 clock selected */
#define RCU_CKOUT0SRC_EXT1              CFG0_CKOUT0SEL(10)                  /*!< EXT1 selected */
#define RCU_CKOUT0SRC_CKPLL2            CFG0_CKOUT0SEL(11)                  /*!< CK_PLL2 clock selected */

/* RTC clock entry selection */
#define BDCTL_RTCSRC(regval)            (BITS(8,9) & ((uint32_t)(regval) << 8))
#define RCU_RTCSRC_NONE                 BDCTL_RTCSRC(0)                     /*!< no clock selected */
#define RCU_RTCSRC_LXTAL                BDCTL_RTCSRC(1)                     /*!< RTC source clock select LXTAL  */
#define RCU_RTCSRC_IRC40K               BDCTL_RTCSRC(2)                     /*!< RTC source clock select IRC40K */
#define RCU_RTCSRC_HXTAL_DIV_128        BDCTL_RTCSRC(3)                     /*!< RTC source clock select HXTAL/128 */

/* PREDV0 division factor */
#define CFG1_PREDV0(regval)             (BITS(0,3) & ((uint32_t)(regval) << 0))
#define RCU_PREDV0_DIV1                CFG1_PREDV0(0)                       /*!< PREDV0 input source clock not divided */
#define RCU_PREDV0_DIV2                CFG1_PREDV0(1)                       /*!< PREDV0 input source clock divided by 2 */
#define RCU_PREDV0_DIV3                CFG1_PREDV0(2)                       /*!< PREDV0 input source clock divided by 3 */
#define RCU_PREDV0_DIV4                CFG1_PREDV0(3)                       /*!< PREDV0 input source clock divided by 4 */
#define RCU_PREDV0_DIV5                CFG1_PREDV0(4)                       /*!< PREDV0 input source clock divided by 5 */
#define RCU_PREDV0_DIV6                CFG1_PREDV0(5)                       /*!< PREDV0 input source clock divided by 6 */
#define RCU_PREDV0_DIV7                CFG1_PREDV0(6)                       /*!< PREDV0 input source clock divided by 7 */
#define RCU_PREDV0_DIV8                CFG1_PREDV0(7)                       /*!< PREDV0 input source clock divided by 8 */
#define RCU_PREDV0_DIV9                CFG1_PREDV0(8)                       /*!< PREDV0 input source clock divided by 9 */
#define RCU_PREDV0_DIV10               CFG1_PREDV0(9)                       /*!< PREDV0 input source clock divided by 10 */
#define RCU_PREDV0_DIV11               CFG1_PREDV0(10)                      /*!< PREDV0 input source clock divided by 11 */
#define RCU_PREDV0_DIV12               CFG1_PREDV0(11)                      /*!< PREDV0 input source clock divided by 12 */
#define RCU_PREDV0_DIV13               CFG1_PREDV0(12)                      /*!< PREDV0 input source clock divided by 13 */
#define RCU_PREDV0_DIV14               CFG1_PREDV0(13)                      /*!< PREDV0 input source clock divided by 14 */
#define RCU_PREDV0_DIV15               CFG1_PREDV0(14)                      /*!< PREDV0 input source clock divided by 15 */
#define RCU_PREDV0_DIV16               CFG1_PREDV0(15)                      /*!< PREDV0 input source clock divided by 16 */

/* PREDV1 division factor */
#define CFG1_PREDV1(regval)             (BITS(4,7) & ((uint32_t)(regval) << 4))
#define RCU_PREDV1_DIV1                CFG1_PREDV1(0)                       /*!< PREDV1 input source clock not divided */
#define RCU_PREDV1_DIV2                CFG1_PREDV1(1)                       /*!< PREDV1 input source clock divided by 2 */
#define RCU_PREDV1_DIV3                CFG1_PREDV1(2)                       /*!< PREDV1 input source clock divided by 3 */
#define RCU_PREDV1_DIV4                CFG1_PREDV1(3)                       /*!< PREDV1 input source clock divided by 4 */
#define RCU_PREDV1_DIV5                CFG1_PREDV1(4)                       /*!< PREDV1 input source clock divided by 5 */
#define RCU_PREDV1_DIV6                CFG1_PREDV1(5)                       /*!< PREDV1 input source clock divided by 6 */
#define RCU_PREDV1_DIV7                CFG1_PREDV1(6)                       /*!< PREDV1 input source clock divided by 7 */
#define RCU_PREDV1_DIV8                CFG1_PREDV1(7)                       /*!< PREDV1 input source clock divided by 8 */
#define RCU_PREDV1_DIV9                CFG1_PREDV1(8)                       /*!< PREDV1 input source clock divided by 9 */
#define RCU_PREDV1_DIV10               CFG1_PREDV1(9)                       /*!< PREDV1 input source clock divided by 10 */
#define RCU_PREDV1_DIV11               CFG1_PREDV1(10)                      /*!< PREDV1 input source clock divided by 11 */
#define RCU_PREDV1_DIV12               CFG1_PREDV1(11)                      /*!< PREDV1 input source clock divided by 12 */
#define RCU_PREDV1_DIV13               CFG1_PREDV1(12)                      /*!< PREDV1 input source clock divided by 13 */
#define RCU_PREDV1_DIV14               CFG1_PREDV1(13)                      /*!< PREDV1 input source clock divided by 14 */
#define RCU_PREDV1_DIV15               CFG1_PREDV1(14)                      /*!< PREDV1 input source clock divided by 15 */
#define RCU_PREDV1_DIV16               CFG1_PREDV1(15)                      /*!< PREDV1 input source clock divided by 16 */

/* PLL1 clock multiplication factor */
#define CFG1_PLL1MF(regval)             (BITS(8,11) & ((uint32_t)(regval) << 8))
#define RCU_PLL1_MUL8                   CFG1_PLL1MF(6)                      /*!< PLL1 source clock multiply by 8 */
#define RCU_PLL1_MUL9                   CFG1_PLL1MF(7)                      /*!< PLL1 source clock multiply by 9 */
#define RCU_PLL1_MUL10                  CFG1_PLL1MF(8)                      /*!< PLL1 source clock multiply by 10 */
#define RCU_PLL1_MUL11                  CFG1_PLL1MF(9)                      /*!< PLL1 source clock multiply by 11 */
#define RCU_PLL1_MUL12                  CFG1_PLL1MF(10)                     /*!< PLL1 source clock multiply by 12 */
#define RCU_PLL1_MUL13                  CFG1_PLL1MF(11)                     /*!< PLL1 source clock multiply by 13 */
#define RCU_PLL1_MUL14                  CFG1_PLL1MF(12)                     /*!< PLL1 source clock multiply by 14 */
#define RCU_PLL1_MUL15                  CFG1_PLL1MF(13)                     /*!< PLL1 source clock multiply by 15 */
#define RCU_PLL1_MUL16                  CFG1_PLL1MF(14)                     /*!< PLL1 source clock multiply by 16 */
#define RCU_PLL1_MUL20                  CFG1_PLL1MF(15)                     /*!< PLL1 source clock multiply by 20 */

/* PLL2 clock multiplication factor */
#define CFG1_PLL2MF(regval)             (BITS(12,15) & ((uint32_t)(regval) << 12))
#define RCU_PLL2_MUL8                   CFG1_PLL2MF(6)                      /*!< PLL2 source clock multiply by 8 */
#define RCU_PLL2_MUL9                   CFG1_PLL2MF(7)                      /*!< PLL2 source clock multiply by 9 */
#define RCU_PLL2_MUL10                  CFG1_PLL2MF(8)                      /*!< PLL2 source clock multiply by 10 */
#define RCU_PLL2_MUL11                  CFG1_PLL2MF(9)                      /*!< PLL2 source clock multiply by 11 */
#define RCU_PLL2_MUL12                  CFG1_PLL2MF(10)                     /*!< PLL2 source clock multiply by 12 */
#define RCU_PLL2_MUL13                  CFG1_PLL2MF(11)                     /*!< PLL2 source clock multiply by 13 */
#define RCU_PLL2_MUL14                  CFG1_PLL2MF(12)                     /*!< PLL2 source clock multiply by 14 */
#define RCU_PLL2_MUL15                  CFG1_PLL2MF(13)                     /*!< PLL2 source clock multiply by 15 */
#define RCU_PLL2_MUL16                  CFG1_PLL2MF(14)                     /*!< PLL2 source clock multiply by 16 */
#define RCU_PLL2_MUL20                  CFG1_PLL2MF(15)                     /*!< PLL2 source clock multiply by 20 */


/* PREDV0 input clock source selection */
#define RCU_PREDV0SRC_HXTAL             ((uint32_t)0x00000000U)             /*!< HXTAL selected as PREDV0 input source clock */
#define RCU_PREDV0SRC_CKPLL1            RCU_CFG1_PREDV0SEL                  /*!< CK_PLL1 selected as PREDV0 input source clock */

/* I2S1 clock source selection */
#define RCU_I2S1SRC_CKSYS               ((uint32_t)0x00000000U)             /*!< system clock selected as I2S1 source clock */
#define RCU_I2S1SRC_CKPLL2_MUL2         RCU_CFG1_I2S1SEL                    /*!< (CK_PLL2 x 2) selected as I2S1 source clock */

/* I2S2 clock source selection */
#define RCU_I2S2SRC_CKSYS               ((uint32_t)0x00000000U)             /*!< system clock selected as I2S2 source clock */
#define RCU_I2S2SRC_CKPLL2_MUL2         RCU_CFG1_I2S2SEL                    /*!< (CK_PLL2 x 2) selected as I2S2 source clock */


/* deep-sleep mode voltage */
#define DSV_DSLPVS(regval)              (BITS(0,1) & ((uint32_t)(regval) << 0))
#define RCU_DEEPSLEEP_V_1_2             DSV_DSLPVS(0)                       /*!< core voltage is 1.2V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_1_1             DSV_DSLPVS(1)                       /*!< core voltage is 1.1V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_1_0             DSV_DSLPVS(2)                       /*!< core voltage is 1.0V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_0_9             DSV_DSLPVS(3)  


namespace miosix {

/**
 * Serial port class for GD32 microcontrollers.
 * 
 * Classes of this type are reference counted, must be allocated on the heap
 * and managed through intrusive_ref_ptr<FileBase>
 */
class GD32Serial : public Device
{
public:
    /**
     * Constructor, initializes the serial port.
     * Calls errorHandler(UNEXPECTED) if id is not in the correct range, or when
     * attempting to construct multiple objects with the same id. That is,
     * it is possible to instantiate only one instance of this class for each
     * hardware USART.
     * \param id 0=USART0, 1=USART1
     * \param baudrate serial port baudrate.
     */
    GD32Serial(int id, int baudrate);

    /**
     * Read a block of data
     * \param buffer buffer where read data will be stored
     * \param size buffer size
     * \param where where to read from
     * \return number of bytes read or a negative number on failure
     */
    ssize_t readBlock(uint16_t *buffer, size_t size, off_t where);

    /**
     * Write a block of data
     * \param buffer buffer where take data to write
     * \param size buffer size
     * \param where where to write to
     * \return number of bytes written or a negative number on failure
     */
    ssize_t writeBlock(uint32_t *buffer, size_t size, off_t where);
};

} //namespace miosix

#endif //SERIAL_GD32_H
