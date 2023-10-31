/**
 * @file main.cpp
 * @brief MCP2515 example for STM32F446RE based on LibOpenCM3 in C.
 * @author ZhengKF (minchen9292@gmail.com)
 */
/* SELECT MODE */
#define SEND_MODE
// #define READ_MODE
// #define IRQ_MODE
// #define TEST_MODE

#include <stdio.h>
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/stm32/exti.h"
#include "mcp2515.h"

#include "mcp2515_communication.h"

mcp2515_handle_t mcp2515;

can_frame_t Receieve_frame;
int SendTimes = 0;

int main(void)
{
  mcp2515_setup();
  usart2_setup();
      printf("INIT SUCCESS\r\n");
  mcp2515_make_handle(&mcp2515_select, &mcp2515_deselect, &mcp2515_spi_transfer, &mcp2515_delay_ms, &mcp2515);
  if (mcp2515_init(&mcp2515))
  {
    printf("INIT SUCCESS\r\n");
  }

  while (1)
  {
#if defined(SEND_MODE)
    if (SendTimes % 10 == 0 && SendTimes > 0)
    {
      printf("Send %d times", SendTimes);
      printf(" \r\n");
    }
    mcp2515_sendMessage(&mcp2515, &tx_frame_1);
    mcp2515_print_can_frame(tx_frame_1);
    mcp2515_delay_ms(100);

    mcp2515_sendMessage(&mcp2515, &tx_frame_2);
    mcp2515_print_can_frame(tx_frame_2);
    mcp2515_delay_ms(100);
    SendTimes += 1;
#elif defined(READ_MODE)
    if (mcp2515_readMessage(&mcp2515, &Receieve_frame) == ERROR_OK)
    {
      if (Receieve_frame.can_id == 0 && Receieve_frame.can_dlc == 0)
      {
        printf(".");  // Bus line is empty
      }
      else
      {
        mcp2515_print_can_frame(Receieve_frame);
      }
    }
    else
    {
      printf("mcp2515_readMessage FAILED\r\n");
    }
    mcp2515_delay_ms(100);
#elif defined(IRQ_MODE)

    /* 此模式全部再exti裡面完成，主程式不會需要任何程式 */

#elif defined(TEST_MODE)

#endif
  }  // while(1)
}  // main

/* INT pin interrupt handler. */
void exti9_5_isr(void)
{
  printf("=");
  printf("\r\n");
  can_frame_t rx_frame;
  uint8_t irq = mcp2515_getInterrupts(&mcp2515);
  if (irq & CANINTF_RX0IF)
  {
    if (mcp2515_readMessage_RXBn(&mcp2515, RXB0, &rx_frame) == ERROR_OK)
    {
      mcp2515_print_can_frame(rx_frame);
      mcp2515_sendMessage(&mcp2515, &rx_frame);
    }
  }
  if (irq & CANINTF_RX1IF)
  {
    if (mcp2515_readMessage_RXBn(&mcp2515, RXB1, &rx_frame) == ERROR_OK)
    {
      mcp2515_print_can_frame(rx_frame);
      mcp2515_sendMessage(&mcp2515, &rx_frame);
    }
  }
  if (irq & CANINTF_TX0IF)
  {
    printf("TX0IF");
    printf("\r\n");
  }
  else if (irq & CANINTF_TX1IF)
  {
    printf("TX1IF");
    printf("\r\n");
  }
  else if (irq & CANINTF_TX2IF)
  {
    printf("TX2IF");
    printf("\r\n");
  }
  exti_reset_request(INT_EXTI);
}
