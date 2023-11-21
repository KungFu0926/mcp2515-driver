#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/tpiu.h>
#include <libopencm3/cm3/itm.h>
#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/stm32/usart.h>

#include "default_mode.h"

#define TPIU_SPPR_MASK (0x3)

#define ETM_BASE (PPBI_BASE + 0x41000)
#define ETM_CR MMIO32(ETM_BASE + 0x000)
#define ETM_SR MMIO32(ETM_BASE + 0x010)
#define ETM_TER MMIO32(ETM_BASE + 0x008)
#define ETM_TECR MMIO32(ETM_BASE + 0x01C)
#define ETM_TEER MMIO32(ETM_BASE + 0x020)
#define ETM_TSTART_OR_STOPR MMIO32(ETM_BASE + 0x024)

#define USART_BAUDRATE (115200)

/*--------------------Here for SWO_setup----------------------*/
void swo_enable(void);
void swo_gpio_setup(void);
void dbg_setup(void);
void tpiu_setup(void);
void itm_setup(void);
void etm_setup(void);

/*--------------------Here for common tools---------------------*/
void rcc_setup(void);
void led_setup(void);
void systick_setup(void);
void usart2_setup(void);

void default_mode_start_up()
{
  rcc_setup();
  led_setup();
  systick_setup();
  swo_enable();
  usart2_setup();
}

void rcc_setup(void)
{
  /*
  Using 168MHz as system frequency with 8MHz HSE source
  ahb_frequency  = 168MHz,
  apb1_frequency = 84MHz,
  apb2_frequency = 168MHz,
*/
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
}

void led_setup(void)
{
  /* Set LED pin to output push-pull. */
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO5);
}

void systick_setup(void)
{
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  systick_set_reload(rcc_ahb_frequency / 8 / 1000 - 1);

  systick_counter_enable();
  systick_interrupt_enable();
}

void swo_enable()
{
  swo_gpio_setup();
  dbg_setup();
  tpiu_setup();
  itm_setup();
}

void swo_gpio_setup()
{
  /* PA13 */
  gpio_mode_setup(GPIOA,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO13);

  gpio_set_output_options(GPIOA, GPIO_PUPD_NONE, GPIO_OSPEED_100MHZ, GPIO13);

  gpio_set_af(GPIOA,
              GPIO_AF0,
              GPIO13);

  /* PA14 */
  gpio_mode_setup(GPIOA,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO14);
  gpio_set_output_options(GPIOA, GPIO_PUPD_NONE, GPIO_OSPEED_100MHZ, GPIO14);

  gpio_set_af(GPIOA,
              GPIO_AF0,
              GPIO14);

  /* PB3 */
  gpio_mode_setup(GPIOB,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO3);

  gpio_set_output_options(GPIOB, GPIO_PUPD_NONE, GPIO_OSPEED_100MHZ, GPIO3);

  gpio_set_af(GPIOB,
              GPIO_AF0,
              GPIO3);
}

void dbg_setup()
{
  // Enable Trace IO
  DBGMCU_CR |= DBGMCU_CR_TRACE_IOEN;

  // set debug mode as mode 0 "Asynchronous Mode"
  uint32_t dbmode = DBGMCU_CR;
  dbmode &= ~DBGMCU_CR_TRACE_MODE_MASK;
  dbmode |= DBGMCU_CR_TRACE_MODE_ASYNC;
  DBGMCU_CR = dbmode;
}
void tpiu_setup()
{
  /*
    Configure TPIU
    1. TPIU_CSPSR (Current Port Size Register)
    2. TPIU_FFCR ,0x102 is a default value
    3. SPP_R (Selected Pin Protocol). Using Asynchronous MANCHESTER mode.
    4. CSPS_R (Current port size Register) and choose size 4.
    5. Flash control
        bit 8 always 1
        bit 1 can be set to 1 in asynchronous mode , else reserve 0.
  */

  TPIU_CSPSR |= (0x1 << 0);

  TPIU_FFCR |= (0x102);

  uint32_t sppr = TPIU_SPPR;
  sppr &= ~TPIU_SPPR_MASK;
  sppr |= TPIU_SPPR_ASYNC_MANCHESTER;
  TPIU_SPPR = sppr;

  TPIU_CSPSR |= (0x1 << 3);

  TPIU_FFCR |= TPIU_FFCR_TRIGIN;
  TPIU_FFCR |= TPIU_FFCR_ENFCONT;
}

void itm_setup()
{
  /*
     Configure the ITM
     1. Unlock ITM lock access
     2. Configure TC_R(ITM_TCR)
     3. Configure TE_R(ITM_TER)
     4. Configure TP_R(ITM_TPR)
  */
  ITM_LAR |= CORESIGHT_LAR_KEY;

  uint32_t tcr_config = ITM_TCR;
  tcr_config |= (0x00010005);
  ITM_TCR = tcr_config;

  *ITM_TER |= (0x1 << 0);

  ITM_TPR |= (0x1 << 0);
}

void etm_setup()
{
  ETM_CR |= (0x00001D1E);
  ETM_TER |= (0x0000406F);
  ETM_TEER |= (0x0000006F);
  ETM_TSTART_OR_STOPR |= (0x00000001);
  ETM_CR |= (0x0000191E);
}

/*
  For printf().
  Will emit message to swo and usart2
*/
int _write(int file, char *ptr, int len)
{
  for (int index = 0; index < len; index++)
  {
    usart_send_blocking(USART2, ptr[index]);
  }
  for (int index = 0; index < len; index++)
  {
    ITM_SendChar(*ptr++);
  }

  return len;
}

uint32_t ITM_SendChar(uint32_t ch)
{
  if ((ITM_TCR & ITM_TCR_ITMENA) != 0UL &&
      (*ITM_TER & (1ul << 0)) != 0UL)
  {
    while (ITM_STIM32(0) == 0UL)
    {
      __asm__("nop"); /* Do nothing. */
    }
    ITM_STIM8(0) = (uint8_t)ch;
  }
  return ch;
}

void usart2_setup(void)
{
  rcc_periph_clock_enable(RCC_USART2);
  /* Set USART-Tx pin to alternate function. */
  gpio_mode_setup(GPIOA,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO2 | GPIO3);

  gpio_set_af(GPIOA,
              GPIO_AF7,
              GPIO2 | GPIO3);

  /* Setup interrupt. */
  nvic_enable_irq(NVIC_USART2_IRQ);
  usart_enable_rx_interrupt(USART2); /* Enable receive interrupt. */

  /* Config USART params. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX); /* Tx-Only mode. */

  usart_enable(USART2);
}