
/**
 * @file mcp2515_communication.h
 * @brief Communication interface between STM32 and mcp2515.
 * @author ZhengKF (nfu202208@gmail.com)
 * @copyright MIT License.
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "libopencm3/stm32/rcc.h"
#include "libopencm3/cm3/nvic.h"
#include "can.h"
#include "mcp2515.h"

/* SPI1 */
#define RCC_SPI_PORT (GPIOA)
#define RCC_SPI_CS_PORT (GPIOB)

#define SPI_SCK_PIN (GPIO5)  /* Arduino-D13 pin. */
#define SPI_MISO_PIN (GPIO6) /* Arduino-D12 pin. */
#define SPI_MOSI_PIN (GPIO7) /* Arduino-D11 pin. */
#define SPI_CS_PIN (GPIO6)   /* Arduino-D10 pin. */
#define SPI_AF (GPIO_AF5)

#define INT_PORT (GPIOC)
#define INT_PIN (GPIO7) /* Arduino-D9 pin. */
#define INT_EXTI (EXTI7)
#define INT_IRQ (NVIC_EXTI9_5_IRQ)

void mcp2515_pin_setup(void);
bool mcp2515_init(const mcp2515_handle_t *mcp2515_handle);

void mcp2515_deselect(void);
void mcp2515_select(void);
uint8_t mcp2515_spi_transfer(uint8_t data);
void mcp2515_delay_ms(uint32_t ms);

void mcp2515_print_can_frame(can_frame_t *can_frame);
bool mcp2515_compare_frame(can_frame_t *frame1, can_frame_t *frame2);

extern can_frame_t tx_frame_1;
extern can_frame_t tx_frame_2;
#endif