/**
 * @file default_mode.h
 * @brief Provide a sample for STM32 nucleo F446RE develope with
 *        1. Rcc in 168MHz (using "HSE")
 *        2. timer4 (for delay)
 *        3. usart2 TX & RX
 *        4. LED
 *        5. SWO
 *        All you have to do is call "default_mode_start_up()" to enable the
 *        default mode, and copy the "tim4_isr(void)" to main to enable the delay function.
 * @author ZhengKF (nfu202208@gmail.com)
 * @copyright MIT License.
 */
#ifndef COMMON_TOOL_H_
  #define COMMON_TOOL_H_

  #include <stdint.h>

  #define GPIO_LED_PORT (GPIOA)
  #define GPIO_LED_PIN (GPIO5) /* D13. */

  #define ON_BOARD_BUTTON_PORT (GPIOC)
  #define ON_BOARD_BUTTON_PIN (GPIO13)
typedef enum
{
  S,
  MS,
  US,
} delayUnit;

void default_mode_start_up(void);

int _write(int file, char *ptr, int len);
uint32_t ITM_SendChar(uint32_t ch);
void delay(uint32_t value, delayUnit unit);

extern volatile uint32_t counter;
#endif

/*------------------ below is timer4(for delay,resolutin 1us) and timer7 (for timepass counter, resolution 100ms )------------------*/

// volatile uint32_t counter = 0;

// void tim4_isr(void)
// {
//   /* Check 'Update interrupt flag'. */
//   counter += 1;
//   if (timer_get_flag(TIM4, TIM_SR_UIF))
//   {
//     timer_clear_flag(TIM4, TIM_SR_UIF);
//   }
// }

// /**
//  * @brief Timer7 Interrupt service routine.
//  */
// void tim7_isr(void)
// {
//   if (timer_get_flag(TIM7, TIM_SR_UIF))
//   {
//     timer_clear_flag(TIM7, TIM_SR_UIF);
//     // gpio_toggle(GPIO_LED_PORT, GPIO_LED_PIN); /* LED on/off. */
//   }
// }