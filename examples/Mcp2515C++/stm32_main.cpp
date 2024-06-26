// /**
//  * @file main.cpp
//  * @brief MCP2515 example for STM32F446RE based on LibOpenCM3.
//  * @author ZiTe (honmonoh@gmail.com)
//  */

// #include "libopencm3/stm32/rcc.h"
// #include "libopencm3/stm32/gpio.h"
// #include "libopencm3/stm32/spi.h"
// #include "libopencm3/stm32/exti.h"
// #include "libopencm3/cm3/nvic.h"
// #include "mcp2515.h"

// #define SPI_PORT (GPIOA)
// #define SPI_SCK_PIN (GPIO5)  /* Arduino-D13 pin. */
// #define SPI_MISO_PIN (GPIO6) /* Arduino-D12 pin. */
// #define SPI_MOSI_PIN (GPIO7) /* Arduino-D11 pin. */
// #define SPI_CS_PIN (GPIO4)   /* Arduino-A2 pin. */
// #define SPI_AF (GPIO_AF5)

// #define INT_PORT (GPIOC)
// #define INT_PIN (GPIO7) /* Arduino-D9 pin. */
// #define INT_EXTI (EXTI7)
// #define INT_IRQ (NVIC_EXTI9_5_IRQ)

// MCP2515 *mcp2515;

// static void rcc_setup(void)
// {
//   rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
//   rcc_periph_clock_enable(RCC_GPIOA);
//   rcc_periph_clock_enable(RCC_GPIOC);
//   rcc_periph_clock_enable(RCC_SPI1);
//   rcc_periph_clock_enable(RCC_SYSCFG); /* For EXTI. */
// }

// static void spi_setup(void)
// {
//   gpio_mode_setup(SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_CS_PIN);
//   gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN);
//   gpio_set_af(SPI_PORT, SPI_AF, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN);
//   gpio_set_output_options(SPI_PORT,
//                           GPIO_OTYPE_PP,
//                           GPIO_OSPEED_50MHZ,
//                           SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN | SPI_CS_PIN);

//   gpio_set(SPI_PORT, SPI_CS_PIN); /* Deselect. */

//   uint32_t spi = SPI1;
//   spi_disable(spi);
//   spi_reset(spi);

//   spi_init_master(spi,
//                   SPI_CR1_BAUDRATE_FPCLK_DIV_32,   /* Max 10 MHz. */
//                   SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, /* CPOL=0. */
//                   SPI_CR1_CPHA_CLK_TRANSITION_1,   /* CPHA=0. */
//                   SPI_CR1_DFF_8BIT,                /* Data frame 8-bit. */
//                   SPI_CR1_MSBFIRST);               /* Order: MSB First. */

//   spi_set_full_duplex_mode(spi);

//   spi_enable_software_slave_management(spi);
//   spi_set_nss_high(spi);

//   spi_enable(spi);
// }

// static void int_pin_setup(void)
// {
//   gpio_mode_setup(INT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, INT_PIN);
//   exti_select_source(INT_EXTI, INT_PORT);
//   exti_set_trigger(INT_EXTI, EXTI_TRIGGER_FALLING);
//   exti_enable_request(INT_EXTI);
//   nvic_enable_irq(INT_IRQ);
// }

// void mcp2515_select(void)
// {
//   gpio_clear(SPI_PORT, SPI_CS_PIN); /* CS pin output low to select. */
// }

// void mcp2515_deselect(void)
// {
//   while (!(SPI_SR(SPI1) & SPI_SR_TXE)) /* Wait for 'Transmit buffer empty' flag to set. */
//   {
//   }
//   while ((SPI_SR(SPI1) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
//   {
//   }

//   gpio_set(SPI_PORT, SPI_CS_PIN); /* CS pin output high to deselect. */
// }

// uint8_t mcp2515_spi_transfer(uint8_t data)
// {
//   uint16_t rec = spi_xfer(SPI1, data);
//   return rec & 0xFF;
// }

// void delay(uint32_t ms)
// {
//   for (; ms > 0; ms--)
//   {
//     for (uint16_t j = 0; j < 25000; j++)
//     {
//       __asm__("nop"); /* Do nothing. */
//     }
//   }
// }

// int main(void)
// {
//   rcc_setup();
//   spi_setup();
//   int_pin_setup();

//   mcp2515 = new MCP2515(&mcp2515_select,
//                         &mcp2515_deselect,
//                         &mcp2515_spi_transfer,
//                         &delay);
//   mcp2515->reset();
//   mcp2515->setBitrate(CAN_125KBPS, MCP_8MHZ);
//   mcp2515->setNormalMode();

//   can_frame tx_frame_1;
//   can_frame tx_frame_2;

//   tx_frame_1.can_id = 0x000;
//   tx_frame_1.can_dlc = 8;
//   tx_frame_1.data[0] = 0x01;
//   tx_frame_1.data[1] = 0x02;
//   tx_frame_1.data[2] = 0x03;
//   tx_frame_1.data[3] = 0x04;
//   tx_frame_1.data[4] = 0x05;
//   tx_frame_1.data[5] = 0x06;
//   tx_frame_1.data[6] = 0x07;
//   tx_frame_1.data[7] = 0x08;

//   tx_frame_2.can_id = 0x001;
//   tx_frame_2.can_dlc = 8;
//   tx_frame_2.data[0] = 0xF1;
//   tx_frame_2.data[1] = 0xF2;
//   tx_frame_2.data[2] = 0xF3;
//   tx_frame_2.data[3] = 0xF4;
//   tx_frame_2.data[4] = 0xF5;
//   tx_frame_2.data[5] = 0xF6;
//   tx_frame_2.data[6] = 0xF7;
//   tx_frame_2.data[7] = 0xF8;

//   while (1)
//   {
//     mcp2515->sendMessage(&tx_frame_1);
//     delay(1000);

//     mcp2515->sendMessage(&tx_frame_2);
//     delay(1000);
//   }

//   return 0;
// }

// /* INT pin interrupt handler. */
// void exti9_5_isr(void)
// {
//   can_frame rx_frame;
//   uint8_t irq = mcp2515->getInterrupts();

//   if (irq & MCP2515::CANINTF_RX0IF)
//   {
//     if (mcp2515->readMessage(MCP2515::RXB0, &rx_frame) == MCP2515::ERROR_OK)
//     {
//       rx_frame.can_id = 0x0F0;
//       mcp2515->sendMessage(&rx_frame);
//     }
//   }

//   if (irq & MCP2515::CANINTF_RX1IF)
//   {
//     if (mcp2515->readMessage(MCP2515::RXB1, &rx_frame) == MCP2515::ERROR_OK)
//     {
//       rx_frame.can_id = 0x0F1;
//       mcp2515->sendMessage(&rx_frame);
//     }
//   }

//   exti_reset_request(INT_EXTI);
// }
