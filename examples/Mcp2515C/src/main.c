/**
 * @file main.cpp
 * @brief MCP2515 example for STM32F446RE based on LibOpenCM3 in C.
 * @author ZhengKF (minchen9292@gmail.com)
 */
/* SELECT MODE */
// #define SEND_MODE
#define READ_MODE
// #define IRQ_MODE
// #define TEST_MODE

#include <stdio.h>
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/stm32/exti.h"
#include "libopencm3/stm32/gpio.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#include "mcp2515.h"
#include "mcp2515_communication.h"
#include "default_mode.h"


mcp2515_handle_t mcp2515;
can_frame_t Receieve_frame;
int SendTimes = 0;

volatile uint32_t counter = 0;

int num = 0;
int main(void)
{
  default_mode_start_up();

  mcp2515_pin_setup();
  mcp2515_make_handle(&mcp2515_select, &mcp2515_deselect, &mcp2515_spi_transfer, &mcp2515_delay_ms, &mcp2515);
  mcp2515_init(&mcp2515) ? printf("INIT SUCCESS\n") : printf("INIT Failed\n");

  while (1)
  {
#if defined(SEND_MODE)

    if (SendTimes % 10 == 0 && SendTimes > 0)
    {
      printf("Send %d times", SendTimes);
      printf(" \r\n");
    }
    mcp2515_sendMessage(&mcp2515, &tx_frame_1);
    mcp2515_print_can_frame(&tx_frame_1);
    mcp2515_delay_ms(1);
    mcp2515_clearTXn_Interrupts(&mcp2515);

    delay(2, S);

    SendTimes += 1;
#elif defined(READ_MODE)
    if (mcp2515_readMessage(&mcp2515, &Receieve_frame) == ERROR_OK)
    {
      if (Receieve_frame.can_id == 0 && Receieve_frame.can_dlc == 0)
      {
        printf(".");  // Bus line is empty
      }

      else if (Receieve_frame.can_id == 0X36)
      {
        mcp2515_print_can_frame(&Receieve_frame);
      }
    }
    else
    {
      printf("mcp2515_readMessage FAILED\r\n");
    }
    mcp2515_delay_ms(100);
#elif defined(IRQ_MODE)

    /*
      此模式全部再exti裡面完成，主程式不會需要任何程式

      作為接收端時:
      有RX0IE和RX1IE兩個暫存器可以暫時存放can_frame
      可以使用mcp2515_readMessage或mcp2515_readMessage_RXBn讀取暫存器內的can_frame
      使用上述兩個function讀取完資料後不用手動清除RX0IE和RX1IE
      如果傳送端太快導致來不及read的話，這個暫存器會overflow，這時候就需要手動清除整個暫存器使用mcp2515_clearRXnOVRs，來清除
      接收端最低可以接受10ms一次的連續訊息

      作為傳送端時:
      有三個暫存器位置可以存放要傳送的frame TX0IE TX1IE TX2IE
      傳送完frame需要主動清除CANINTF裡的flag
      放在irq內需要一點點的delay才清除的了,實測mcp2515_delay_ms(1)就夠了
    */
#elif defined(TEST_MODE)
    printf("Init=%u\n", mcp2515_getInterrupts(&mcp2515));

    if (mcp2515_checkError(&mcp2515))
    {
      uint8_t err = mcp2515_getErrorFlags(&mcp2515);
      if ((err & EFLG_RX1OVR) || err & EFLG_RX0OVR)
      {
        mcp2515_clearRXnOVRFlags(&mcp2515);
      }
      else
        printf("Error frame\n");
    }
    else
    {
      if (SendTimes % 10 == 0 && SendTimes > 0)
      {
        printf("Send %d times", SendTimes);
        printf(" \r\n");
      }
      mcp2515_sendMessage(&mcp2515, &tx_frame_1);
      mcp2515_print_can_frame(tx_frame_1);
      mcp2515_clearTXn_Interrupts(&mcp2515);
    }

    delay(3, S);

    SendTimes += 1;
#endif
  }  // while(1)
}  // main

/* INT pin interrupt handler. */
void exti9_5_isr(void)
{
  printf("num=%d\n", num++);
  can_frame_t rx_frame;
  uint8_t irq = mcp2515_getInterrupts(&mcp2515);
  printf("---------------%u\n", irq);

  if (irq & CANINTF_RX0IF)
  {
    printf("RX0IF\n");

    if (mcp2515_readMessage_RXBn(&mcp2515, RXB0, &rx_frame) == ERROR_OK)
    {
      mcp2515_print_can_frame(&rx_frame);
      mcp2515_sendMessage(&mcp2515, &rx_frame);
      mcp2515_delay_ms(1);
      mcp2515_clearTXn_Interrupts(&mcp2515);
    }
  }
  else if (irq & CANINTF_RX1IF)
  {
    printf("RX1IF\n");
    if (mcp2515_readMessage_RXBn(&mcp2515, RXB1, &rx_frame) == ERROR_OK)
    {
      mcp2515_print_can_frame(&rx_frame);
      mcp2515_sendMessage(&mcp2515, &rx_frame);
      mcp2515_delay_ms(1);
      mcp2515_clearTXn_Interrupts(&mcp2515);
    }
  }

  if (irq & CANINTF_TX0IF)
  {
    printf("TX0IF\n");
  }
  else if (irq & CANINTF_TX1IF)
  {
    printf("TX1IF\n");
  }
  else if (irq & CANINTF_TX2IF)
  {
    printf("TX2IF\n");
  }

  exti_reset_request(INT_EXTI);
}

void tim4_isr(void)
{
  /* Check 'Update interrupt flag'. */
  counter += 1;
  if (timer_get_flag(TIM4, TIM_SR_UIF))
  {
    timer_clear_flag(TIM4, TIM_SR_UIF);
  }
}