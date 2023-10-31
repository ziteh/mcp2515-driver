
/**
 * @file mcp2515_communication.h
 * @brief Communication interface between STM32 and mcp2515.
 *        Content "SPI1" and "USART2"
 * @author ZhengKF (nfu202208@gmail.com)
 * @copyright MIT License.
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "libopencm3/stm32/rcc.h"
#include "libopencm3/cm3/nvic.h"
#include "can.h"
#include "mcp2515.h"

/* USART2 */
#define USART_BAUDRATE (115200)
#define RCC_USART2_TXRX_PORT (RCC_GPIOA)
#define RCC_ENABLE_USART2 (RCC_USART2)
#define USART2_TXRX_PORT (GPIOA)
#define GPIO_USART2_TX_PIN (GPIO2) /* Arduino-D1. */
#define GPIO_USART2_RX_PIN (GPIO3) /* Arduino-D0. */
#define GPIO_USART2_AF (GPIO_AF7)  /* Table-11 in DS10693 */

/* SPI1 */
#define RCC_SPI_PORT (GPIOA)
#define SPI_SCK_PIN (GPIO5)  /* Arduino-D13 pin. */
#define SPI_MISO_PIN (GPIO6) /* Arduino-D12 pin. */
#define SPI_MOSI_PIN (GPIO7) /* Arduino-D11 pin. */
#define SPI_CS_PIN (GPIO9)   /* Arduino-D8 pin. */
#define SPI_AF (GPIO_AF5)

#define INT_PORT (GPIOC)
#define INT_PIN (GPIO7) /* Arduino-D9 pin. */
#define INT_EXTI (EXTI7)
#define INT_IRQ (NVIC_EXTI9_5_IRQ)

void usart2_setup(void);
void mcp2515_setup(void);
bool mcp2515_init(const mcp2515_handle_t *mcp2515_handle);

void mcp2515_deselect(void);
void mcp2515_select(void);
uint8_t mcp2515_spi_transfer(uint8_t data);
void mcp2515_delay_ms(uint32_t ms);
void mcp2515_print_can_frame(can_frame_t can_frame);
bool mcp2515_compare_frame(can_frame_t frame1, can_frame_t frame2);

int _write(int file, char *ptr, int len);

extern can_frame_t tx_frame_1;
extern can_frame_t tx_frame_2;
#endif