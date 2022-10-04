/**
 * @file main.cpp
 * @brief MCP2515 example for STM32F446RE based on LibOpenCM3.
 * @author ZiTe (honmonoh@gmail.com)
 */

#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/spi.h"
#include "mcp2515.h"

#define SPI_SCK_PIN (GPIO5)  /* Arduino-D13 pin. */
#define SPI_MISO_PIN (GPIO6) /* Arduino-D12 pin. */
#define SPI_MOSI_PIN (GPIO7) /* Arduino-D11 pin. */
#define SPI_CS_PIN (GPIO4)   /* Arduino-A2 pin. */
#define SPI_PORT (GPIOA)
#define SPI_AF (GPIO_AF5)

static void rcc_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_SPI1);
}

static void spi_setup(void)
{
  gpio_mode_setup(SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_CS_PIN);
  gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN);
  gpio_set_af(SPI_PORT, SPI_AF, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN);
  gpio_set_output_options(SPI_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_50MHZ,
                          SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN | SPI_CS_PIN);

  gpio_set(SPI_PORT, SPI_CS_PIN); /* Deselect. */

  uint32_t spi = SPI1;
  spi_disable(spi);
  spi_reset(spi);

  spi_init_master(spi,
                  SPI_CR1_BAUDRATE_FPCLK_DIV_32,   /* Max 10 MHz. */
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, /* CPOL=0. */
                  SPI_CR1_CPHA_CLK_TRANSITION_1,   /* CPHA=0. */
                  SPI_CR1_DFF_8BIT,                /* Data frame 8-bit. */
                  SPI_CR1_MSBFIRST);               /* Order: MSB First. */

  spi_set_full_duplex_mode(spi);

  spi_enable_software_slave_management(spi);
  spi_set_nss_high(spi);

  spi_enable(spi);
}

void mcp2515_select(void)
{
  gpio_clear(SPI_PORT, SPI_CS_PIN); /* CS pin output low to select. */
}

void mcp2515_deselect(void)
{
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)) /* Wait for 'Transmit buffer empty' flag to set. */
  {
  }
  while ((SPI_SR(SPI1) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }

  gpio_set(SPI_PORT, SPI_CS_PIN); /* CS pin output high to deselect. */
}

uint8_t mcp2515_spi_transfer(uint8_t data)
{
  uint16_t rec = spi_xfer(SPI1, data);
  return rec & 0xFF;
}

void delay(uint32_t ms)
{
  for (; ms > 0; ms--)
  {
    for (uint16_t j = 0; j < 25000; j++)
    {
      __asm__("nop"); /* Do nothing. */
    }
  }
}

int main(void)
{
  rcc_setup();
  spi_setup();

  MCP2515 mcp2515(&mcp2515_select,
                  &mcp2515_deselect,
                  &mcp2515_spi_transfer,
                  &delay);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  can_frame msg1;
  can_frame msg2;

  msg1.can_id = 0x000;
  msg1.can_dlc = 8;
  msg1.data[0] = 0x01;
  msg1.data[1] = 0x02;
  msg1.data[2] = 0x03;
  msg1.data[3] = 0x04;
  msg1.data[4] = 0x05;
  msg1.data[5] = 0x06;
  msg1.data[6] = 0x07;
  msg1.data[7] = 0x08;

  msg2.can_id = 0x001;
  msg2.can_dlc = 8;
  msg2.data[0] = 0xF1;
  msg2.data[1] = 0xF2;
  msg2.data[2] = 0xF3;
  msg2.data[3] = 0xF4;
  msg2.data[4] = 0xF5;
  msg2.data[5] = 0xF6;
  msg2.data[6] = 0xF7;
  msg2.data[7] = 0xF8;

  while (1)
  {
    mcp2515.sendMessage(&msg1);
    delay(1000);

    mcp2515.sendMessage(&msg2);
    delay(1000);
  }

  return 0;
}
