/**
 * @file default_mode.h
 * @brief Provide a sample for STM32 nucleo F446RE develope with
 *        1. Rcc in 168MHz (using "HSE")
 *        2. System tick (for delay)
 *        3. usart2 TX & RX
 *        4. LED
 *        5. SWO
 *        All you have to do is call "default_mode_start_up()"" to enable the
 *        default mode.
 * @author ZhengKF (nfu202208@gmail.com)
 * @copyright MIT License.
 */
#ifndef COMMON_TOOL_H_
  #define COMMON_TOOL_H_

  #include <stdint.h>

  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5) /* D13. */

void default_mode_start_up(void);

int _write(int file, char *ptr, int len);
uint32_t ITM_SendChar(uint32_t ch);

#endif

/*------------------------- below is systemtick_delay -------------------------*/
// volatile uint32_t systick_delay = 0;

// void delay_ms(uint32_t ms)
// {
//   systick_delay = ms;
//   while (systick_delay != 0)
//   {
//     /* Wait. */
//   }
// }

// /**
//  * @brief SysTick handler.
//  */
// void sys_tick_handler(void)
// {
//   if (systick_delay != 0)
//   {
//     systick_delay--;
//   }
// }