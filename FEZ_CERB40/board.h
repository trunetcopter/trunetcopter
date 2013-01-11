/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for STMicroelectronics STM32F4-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_FEZ_CERB40
#define BOARD_NAME              "FEZ Cerb40"

/*
 * Board frequencies.
 * NOTE: The LSE crystal is not fitted by default on the board.
 */
#define STM32_LSECLK            0
#define STM32_HSECLK            12000000

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD               300

/*
 * MCU type as defined in the ST header file stm32f4xx.h.
 */
#define STM32F4XX

/*
 * IO pins assignments.
 */
#define GPIOA_TIM2_CH1          0
#define GPIOA_TIM2_CH2          1
#define GPIOA_USART2_TX         2
#define GPIOA_USART2_RX         3
#define GPIOA_SWDIO             13
#define GPIOA_SWCLK             14

#define GPIOB_USART1_TX			6
#define GPIOB_USART1_RX			7
#define GPIOB_SCL               8
#define GPIOB_SDA               9
#define GPIOB_TIM2_CH3          10
#define GPIOB_TIM2_CH4          11
#define GPIOB_OTG_FS_DM         14
#define GPIOB_OTG_FS_DP         15

#define GPIOC_TIM3_CH1          6
#define GPIOC_TIM3_CH2          7
#define GPIOC_TIM3_CH3          8
#define GPIOC_TIM3_CH4          9
#define GPIOC_USART3_TX         10
#define GPIOC_USART3_RX         11

#define GPIOH_OSC_IN            0
#define GPIOH_OSC_OUT           1

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2))
#define PIN_PUDR_FLOATING(n)        (0U << ((n) * 2))
#define PIN_PUDR_PULLUP(n)          (1U << ((n) * 2))
#define PIN_PUDR_PULLDOWN(n)        (2U << ((n) * 2))
#define PIN_AFIO_AF(n, v)           ((v##U) << ((n % 8) * 4))

/*
 * Port A setup.
 * All input with pull-up except:
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ALTERNATE(GPIOA_TIM2_CH1) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_TIM2_CH2) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_USART2_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_USART2_RX) | \
                                     PIN_MODE_INPUT(4) |                   \
                                     PIN_MODE_INPUT(5) |                   \
                                     PIN_MODE_INPUT(6) |                   \
                                     PIN_MODE_INPUT(7) |                   \
                                     PIN_MODE_INPUT(8) |                   \
                                     PIN_MODE_INPUT(9) |                   \
                                     PIN_MODE_INPUT(10) |                  \
                                     PIN_MODE_INPUT(11) |                  \
                                     PIN_MODE_INPUT(12) |                  \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |     \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOA_OTYPER            0x00000000
#define VAL_GPIOA_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOA_PUPDR             (PIN_PUDR_PULLUP(0) |  \
                                     PIN_PUDR_PULLUP(1) |  \
                                     PIN_PUDR_PULLUP(2) |  \
                                     PIN_PUDR_PULLUP(3) |  \
                                     PIN_PUDR_PULLUP(4) |  \
                                     PIN_PUDR_PULLUP(5) |  \
                                     PIN_PUDR_PULLUP(6) |  \
                                     PIN_PUDR_PULLUP(7) |  \
                                     PIN_PUDR_PULLUP(8) |  \
                                     PIN_PUDR_PULLUP(9) |  \
                                     PIN_PUDR_PULLUP(10) | \
                                     PIN_PUDR_PULLUP(11) | \
                                     PIN_PUDR_PULLUP(12) | \
        	                     PIN_PUDR_PULLUP(GPIOA_SWDIO) |   \
	                             PIN_PUDR_PULLDOWN(GPIOA_SWCLK) | \
                                     PIN_PUDR_PULLUP(15))
#define VAL_GPIOA_ODR               0xFFFFFFFF
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_TIM2_CH1, 1) |  \
				     	 	 	 	 PIN_AFIO_AF(GPIOA_TIM2_CH2, 1) |  \
				     	 	 	 	 PIN_AFIO_AF(GPIOA_USART2_TX, 7) | \
				     	 	 	 	 PIN_AFIO_AF(GPIOA_USART2_RX, 7))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_SWDIO, 0) |     \
	                             	 PIN_AFIO_AF(GPIOA_SWCLK, 0))

/*
 * Port B setup.
 * All input with pull-up except:
 * PB6  - GPIOB_SCL             (alternate 4).
 * PB9  - GPIOB_SDA             (alternate 4).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_INPUT(0) |                   \
                                     PIN_MODE_INPUT(1) |                   \
                                     PIN_MODE_INPUT(2) |                   \
                                     PIN_MODE_INPUT(3) |                   \
                                     PIN_MODE_INPUT(4) |                   \
                                     PIN_MODE_INPUT(5) |                   \
                                     PIN_MODE_ALTERNATE(GPIOB_USART1_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOB_USART1_RX) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SCL) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_SDA) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_TIM2_CH3) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_TIM2_CH4) |  \
                                     PIN_MODE_INPUT(12) |                  \
                                     PIN_MODE_INPUT(13) |                  \
                                     PIN_MODE_ALTERNATE(GPIOB_OTG_FS_DM) | \
                                     PIN_MODE_ALTERNATE(GPIOB_OTG_FS_DP))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOB_SCL) |      \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_SDA))
#define VAL_GPIOB_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOB_PUPDR             (PIN_PUDR_PULLUP(0) |                 \
                                     PIN_PUDR_PULLUP(1) |                 \
                                     PIN_PUDR_PULLUP(2) |                 \
                                     PIN_PUDR_PULLUP(3) |                 \
                                     PIN_PUDR_PULLUP(4) |                 \
                                     PIN_PUDR_PULLUP(5) |                 \
                                     PIN_PUDR_PULLUP(6) |                 \
                                     PIN_PUDR_PULLUP(7) |                 \
                                     PIN_PUDR_FLOATING(GPIOB_SCL) |       \
                                     PIN_PUDR_FLOATING(GPIOB_SDA) |       \
                                     PIN_PUDR_PULLUP(GPIOB_TIM2_CH3) |    \
                                     PIN_PUDR_PULLUP(GPIOB_TIM2_CH4) |    \
                                     PIN_PUDR_PULLUP(12) |                \
                                     PIN_PUDR_PULLUP(13) |                \
                                     PIN_PUDR_FLOATING(GPIOB_OTG_FS_DM) | \
                                     PIN_PUDR_FLOATING(GPIOB_OTG_FS_DP))
#define VAL_GPIOB_ODR               0xFFFFFFFF
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_USART1_TX, 7) |  \
				     	 	 	 	 PIN_AFIO_AF(GPIOB_USART1_RX, 7))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_TIM2_CH3, 1) |   \
				     	 	 	 	 PIN_AFIO_AF(GPIOB_TIM2_CH4, 1) |   \
                                     PIN_AFIO_AF(GPIOB_SDA, 4) |        \
                                     PIN_AFIO_AF(GPIOB_SCL, 4) |        \
                                     PIN_AFIO_AF(GPIOB_OTG_FS_DM, 10) | \
                                     PIN_AFIO_AF(GPIOB_OTG_FS_DP, 10))

/*
 * Port C setup.
 * All input with pull-up except:
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(0) |                    \
                                     PIN_MODE_INPUT(1) |                    \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_ALTERNATE(GPIOC_TIM3_CH1) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_TIM3_CH2) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_TIM3_CH3) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_TIM3_CH4) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_USART3_TX) |  \
                                     PIN_MODE_ALTERNATE(GPIOC_USART3_RX) |  \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOC_OTYPER            0x00000000
#define VAL_GPIOC_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOC_PUPDR             (PIN_PUDR_PULLUP(0) |    \
                                     PIN_PUDR_PULLUP(1) |    \
                                     PIN_PUDR_PULLUP(2) |    \
                                     PIN_PUDR_PULLUP(3) |    \
                                     PIN_PUDR_PULLUP(4) |    \
                                     PIN_PUDR_PULLUP(5) |    \
                                     PIN_PUDR_PULLUP(6) |    \
                                     PIN_PUDR_PULLUP(7) |    \
                                     PIN_PUDR_PULLUP(8) |    \
                                     PIN_PUDR_PULLUP(9) |    \
                                     PIN_PUDR_PULLUP(10) |   \
                                     PIN_PUDR_PULLUP(11) |   \
                                     PIN_PUDR_PULLUP(12) |   \
                                     PIN_PUDR_PULLUP(13) |   \
                                     PIN_PUDR_PULLUP(14) |   \
                                     PIN_PUDR_PULLUP(15))
#define VAL_GPIOC_ODR               0xFFFFFFFF
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_TIM3_CH1, 2) |  \
                                     PIN_AFIO_AF(GPIOC_TIM3_CH2, 2))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_TIM3_CH3, 2) |  \
                                     PIN_AFIO_AF(GPIOC_TIM3_CH4, 2) |  \
                                     PIN_AFIO_AF(GPIOC_USART3_TX, 7) | \
                                     PIN_AFIO_AF(GPIOC_USART3_RX, 7))

/*
 * Port D setup.
 * All input with pull-up except:
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(0) |                    \
                                     PIN_MODE_INPUT(1) |                    \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_INPUT(6) |                    \
                                     PIN_MODE_INPUT(7) |                    \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |                   \
                                     PIN_MODE_INPUT(11) |                   \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOD_OTYPER            0x00000000
#define VAL_GPIOD_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOD_PUPDR             (PIN_PUDR_PULLUP(0) |                   \
                                     PIN_PUDR_PULLUP(1) |                   \
                                     PIN_PUDR_PULLUP(2) |                   \
                                     PIN_PUDR_PULLUP(3) |                   \
                                     PIN_PUDR_PULLUP(4) |                   \
                                     PIN_PUDR_PULLUP(5) |                   \
                                     PIN_PUDR_PULLUP(6) |                   \
                                     PIN_PUDR_PULLUP(7) |                   \
                                     PIN_PUDR_PULLUP(8) |                   \
                                     PIN_PUDR_PULLUP(9) |                   \
                                     PIN_PUDR_PULLUP(10) |                  \
                                     PIN_PUDR_PULLUP(11) |                  \
                                     PIN_PUDR_PULLUP(12) |                  \
                                     PIN_PUDR_PULLUP(13) |                  \
                                     PIN_PUDR_PULLUP(14) |                  \
                                     PIN_PUDR_PULLUP(15))
#define VAL_GPIOD_ODR               0xFFFFFFFF
#define VAL_GPIOD_AFRL              0x00000000
#define VAL_GPIOD_AFRH              0x00000000

/*
 * Port H setup.
 * All input with pull-up except:
 * PH0  - GPIOH_OSC_IN          (input floating).
 * PH1  - GPIOH_OSC_OUT         (input floating).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(2) |                    \
                                     PIN_MODE_INPUT(3) |                    \
                                     PIN_MODE_INPUT(4) |                    \
                                     PIN_MODE_INPUT(5) |                    \
                                     PIN_MODE_INPUT(6) |                    \
                                     PIN_MODE_INPUT(7) |                    \
                                     PIN_MODE_INPUT(8) |                    \
                                     PIN_MODE_INPUT(9) |                    \
                                     PIN_MODE_INPUT(10) |                   \
                                     PIN_MODE_INPUT(11) |                   \
                                     PIN_MODE_INPUT(12) |                   \
                                     PIN_MODE_INPUT(13) |                   \
                                     PIN_MODE_INPUT(14) |                   \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOH_OTYPER            0x00000000
#define VAL_GPIOH_OSPEEDR           0xFFFFFFFF
#define VAL_GPIOH_PUPDR             (PIN_PUDR_FLOATING(GPIOH_OSC_IN) |      \
                                     PIN_PUDR_FLOATING(GPIOH_OSC_OUT) |     \
                                     PIN_PUDR_PULLUP(2) |                   \
                                     PIN_PUDR_PULLUP(3) |                   \
                                     PIN_PUDR_PULLUP(4) |                   \
                                     PIN_PUDR_PULLUP(5) |                   \
                                     PIN_PUDR_PULLUP(6) |                   \
                                     PIN_PUDR_PULLUP(7) |                   \
                                     PIN_PUDR_PULLUP(8) |                   \
                                     PIN_PUDR_PULLUP(9) |                   \
                                     PIN_PUDR_PULLUP(10) |                  \
                                     PIN_PUDR_PULLUP(11) |                  \
                                     PIN_PUDR_PULLUP(12) |                  \
                                     PIN_PUDR_PULLUP(13) |                  \
                                     PIN_PUDR_PULLUP(14) |                  \
                                     PIN_PUDR_PULLUP(15))
#define VAL_GPIOH_ODR               0xFFFFFFFF
#define VAL_GPIOH_AFRL              0x00000000
#define VAL_GPIOH_AFRH              0x00000000

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
