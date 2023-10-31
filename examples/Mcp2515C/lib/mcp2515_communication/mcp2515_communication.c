#include <libopencm3/stm32/usart.h>
#include <errno.h>
#include "libopencm3/stm32/gpio.h"
#include <libopencm3/stm32/usart.h>
#include "libopencm3/stm32/spi.h"
#include "libopencm3/stm32/exti.h"
#include "libopencm3/cm3/nvic.h"

#include "mcp2515_communication.h"

void spi_setup(void);
void int_pin_setup(void);

void usart2_setup(void)
{
  rcc_periph_clock_enable(RCC_USART2_TXRX_PORT);
  rcc_periph_clock_enable(RCC_ENABLE_USART2);
  /* Set USART-Tx pin to alternate function. */
  gpio_mode_setup(USART2_TXRX_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_USART2_TX_PIN | GPIO_USART2_RX_PIN);

  gpio_set_af(USART2_TXRX_PORT,
              GPIO_USART2_AF,
              GPIO_USART2_TX_PIN | GPIO_USART2_RX_PIN);

  /* Setup interrupt. */
  nvic_enable_irq(NVIC_USART2_IRQ);
  usart_enable_rx_interrupt(USART2); /* Enable receive interrupt. */

  /* Config USART params. */
  usart_set_baudrate(USART2, USART_BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX);

  usart_enable(USART2);
}

/* For printf(). */
int _write(int file, char *ptr, int len)
{
  int i;
  if (file == 1)
  {
    for (i = 0; i < len; i++)
    {
      usart_send_blocking(USART2, ptr[i]);
    }
    return i;
  }
  errno = EIO;
  return -1;
}

void spi_setup(void)
{
  /*FOR SPI*/
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_SYSCFG); /* For EXTI. */

  gpio_mode_setup(RCC_SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_CS_PIN);
  gpio_mode_setup(RCC_SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN);
  gpio_set_af(RCC_SPI_PORT, SPI_AF,
              SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN);  //
  gpio_set_output_options(RCC_SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                          SPI_SCK_PIN | SPI_MOSI_PIN | SPI_CS_PIN);  //
  gpio_set(RCC_SPI_PORT, SPI_CS_PIN);                                /* Deselect. */
  uint32_t spi = SPI1;
  spi_disable(spi);
  spi_reset(spi);
  spi_init_master(spi, SPI_CR1_BAUDRATE_FPCLK_DIV_32, /* Max 10 MHz. */
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,    /* CPOL=0. */
                  SPI_CR1_CPHA_CLK_TRANSITION_1,      /* CPHA=0. */
                  SPI_CR1_DFF_8BIT,                   /* Data frame 8-bit. */
                  SPI_CR1_MSBFIRST);                  /* Order: MSB First. */
  spi_set_full_duplex_mode(spi);
  spi_enable_software_slave_management(spi);
  spi_set_nss_high(spi);
  mcp2515_deselect();
  spi_enable(spi);
}

void int_pin_setup(void)
{
  gpio_mode_setup(INT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, INT_PIN);
  exti_select_source(INT_EXTI, INT_PORT);
  exti_set_trigger(INT_EXTI, EXTI_TRIGGER_FALLING);
  exti_enable_request(INT_EXTI);
  nvic_enable_irq(INT_IRQ);
}
void mcp2515_setup()
{
  spi_setup();
  int_pin_setup();
}
void mcp2515_deselect()
{
  while (!(SPI_SR(SPI1) &
           SPI_SR_TXE)) /* Wait for 'Transmit buffer empty' flag to set. */
  {
  }
  while ((SPI_SR(SPI1) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }
  gpio_set(RCC_SPI_PORT, SPI_CS_PIN); /* CS pin output high to deselect. */
}

void mcp2515_select()
{
  gpio_clear(RCC_SPI_PORT, SPI_CS_PIN); /* CS pin output low to select. */
}

uint8_t mcp2515_spi_transfer(uint8_t data)
{
  uint16_t rec = spi_xfer(SPI1, data);
  return rec & 0xFF;
}

void mcp2515_delay_ms(uint32_t ms)
{
  for (; ms > 0; ms--)
  {
    for (uint16_t j = 0; j < 25000; j++)
    {
      __asm__("nop"); /* Do nothing. */
    }
  }
}

void mcp2515_print_can_frame(can_frame_t can_frame)
{
  printf("%x ", can_frame.can_id);
  printf("%x ", can_frame.can_dlc);
  /* print the data */
  for (int i = 0; i < can_frame.can_dlc; i++)
  {
    printf("%x ", can_frame.data[i]);
  }
  printf(" \r\n");
}

bool mcp2515_compare_frame(can_frame_t frame1, can_frame_t frame2)
{
  if (frame1.can_id == frame2.can_id &&
      frame1.can_dlc == frame2.can_dlc &&
      frame1.data[0] == frame2.data[0] &&
      frame1.data[1] == frame2.data[1] &&
      frame1.data[2] == frame2.data[2] &&
      frame1.data[3] == frame2.data[3] &&
      frame1.data[4] == frame2.data[4] &&
      frame1.data[5] == frame2.data[5] &&
      frame1.data[6] == frame2.data[6] &&
      frame1.data[7] == frame2.data[7])
  {
    return true;
  }
  else
  {
    return false;
  }
}

can_frame_t tx_frame_1 = {
    .can_id = 0x01,
    .can_dlc = 8,
    .data[0] = 0x01,
    .data[1] = 0x02,
    .data[2] = 0x03,
    .data[3] = 0x04,
    .data[4] = 0x55,
    .data[5] = 0x66,
    .data[6] = 0x77,
    .data[7] = 0x08,
};
can_frame_t tx_frame_2 = {
    .can_id = 0x002,
    .can_dlc = 8,
    .data[0] = 0x11,
    .data[1] = 0x22,
    .data[2] = 0x33,
    .data[3] = 0x44,
    .data[4] = 0x55,
    .data[5] = 0x66,
    .data[6] = 0x77,
    .data[7] = 0x88,
};
bool mcp2515_init(const mcp2515_handle_t *mcp2515_handle)
{
  for (int i = 0; i < 1000; i++)
  {
    if (mcp2515_reset(mcp2515_handle) != ERROR_OK)
    {
      continue;
    }
    if (mcp2515_setBitrate(mcp2515_handle, CAN_125KBPS, MCP_8MHZ) != ERROR_OK)
    {
      continue;
    }
    if (mcp2515_setNormalMode(mcp2515_handle) != ERROR_OK)
    {
      continue;
    }
    return true;
  }
  return false;
}
