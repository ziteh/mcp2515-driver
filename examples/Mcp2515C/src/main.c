/**
 * @file main.cpp
 * @brief MCP2515 example for STM32F446RE based on LibOpenCM3 in C.
 * @author ZhengKF (minchen9292@gmail.com)
 */

#include <stdio.h>
#include <errno.h>
#include "libopencm3/stm32/rcc.h"
#include <libopencm3/stm32/usart.h>
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/spi.h"
#include "libopencm3/stm32/exti.h"
#include "libopencm3/cm3/nvic.h"
#include "mcp2515.h"

#define USART_BAUDRATE (9600)

/* USART2 use the same PORT with SPI1*/
#define RCC_USART_TX_GPIO (RCC_GPIOA)
#define RCC_ENABLE_USART_NUM (RCC_USART2)
#define GPIO_USART_TX_PIN (GPIO2) /* Arduino-D1. */
#define GPIO_USART_AF (GPIO_AF7)  /* Table-11 in DS10693 */
#define CHOSEN_USART_NUM (USART2)

#define SPI_PORT (GPIOA)
#define SPI_SCK_PIN (GPIO5)  /* Arduino-D13 pin. */
#define SPI_MISO_PIN (GPIO6) /* Arduino-D12 pin. */
#define SPI_MOSI_PIN (GPIO7) /* Arduino-D11 pin. */
#define SPI_CS_PIN (GPIO4)   /* Arduino-A2 pin. */
#define SPI_AF (GPIO_AF5)

#define INT_PORT (GPIOC)
#define INT_PIN (GPIO7) /* Arduino-D9 pin. */
#define INT_EXTI (EXTI7)
#define INT_IRQ (NVIC_EXTI9_5_IRQ)

void mcp2515_select(void);
void mcp2515_deselect(void);
uint8_t mcp2515_spi_transfer(uint8_t data);
void mcp2515_delay_ms(uint32_t ms);
int _write(int file, char *ptr, int len);

static void rcc_setup(void)
{
    /*FOR SPI*/
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_SYSCFG); /* For EXTI. */

    /*FOR USART*/
    rcc_periph_clock_enable(RCC_USART_TX_GPIO);
    rcc_periph_clock_enable(RCC_ENABLE_USART_NUM);
}

static void usart_setup(void)
{
    /* Set USART-Tx pin to alternate function. */
    gpio_mode_setup(SPI_PORT,
                    GPIO_MODE_AF,
                    GPIO_PUPD_NONE,
                    GPIO_USART_TX_PIN);

    gpio_set_af(SPI_PORT,
                GPIO_USART_AF,
                GPIO_USART_TX_PIN);

    /* Config USART params. */
    usart_set_baudrate(CHOSEN_USART_NUM, USART_BAUDRATE);
    usart_set_databits(CHOSEN_USART_NUM, 8);
    usart_set_stopbits(CHOSEN_USART_NUM, USART_STOPBITS_1);
    usart_set_parity(CHOSEN_USART_NUM, USART_PARITY_NONE);
    usart_set_flow_control(CHOSEN_USART_NUM, USART_FLOWCONTROL_NONE);
    usart_set_mode(CHOSEN_USART_NUM, USART_MODE_TX); /* Tx-Only mode. */

    usart_enable(CHOSEN_USART_NUM);
}

static void spi_setup(void)
{
    gpio_mode_setup(SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_CS_PIN);

    gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN);

    gpio_set_af(SPI_PORT, SPI_AF, SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN); //
    gpio_set_output_options(SPI_PORT,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_50MHZ,
                            SPI_SCK_PIN | SPI_MOSI_PIN | SPI_CS_PIN); //

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

    mcp2515_deselect();
    spi_enable(spi);
}

static void int_pin_setup(void)
{
    gpio_mode_setup(INT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, INT_PIN);
    exti_select_source(INT_EXTI, INT_PORT);
    exti_set_trigger(INT_EXTI, EXTI_TRIGGER_FALLING);
    exti_enable_request(INT_EXTI);
    nvic_enable_irq(INT_IRQ);
}

mcp2515_handle_t mcp2515;
can_frame_t tx_frame_1 =
    {
        .can_id = 0x000,
        .can_dlc = 8,

        .data[0] = 0x01,
        .data[1] = 0x02,
        .data[2] = 0x03,
        .data[3] = 0x04,
        .data[4] = 0x05,
        .data[5] = 0x06,
        .data[6] = 0x07,
        .data[7] = 0x08,
};
can_frame_t tx_frame_2 =
    {
        .can_id = 0x001,
        .can_dlc = 8,

        .data[0] = 0xF1,
        .data[1] = 0xF2,
        .data[2] = 0xF3,
        .data[3] = 0xF4,
        .data[4] = 0xF5,
        .data[5] = 0xF6,
        .data[6] = 0xF7,
        .data[7] = 0xF8,
};
int main(void)
{
    rcc_setup();
    spi_setup();
    int_pin_setup();
    usart_setup();

    mcp2515_make_handle(&mcp2515_select,
                        &mcp2515_deselect,
                        &mcp2515_spi_transfer,
                        &mcp2515_delay_ms,
                        &mcp2515);

    for (int i = 0; i < 100; i++)
    {
        mcp2515_reset(&mcp2515);
        mcp2515_setBitrate(&mcp2515, CAN_125KBPS, MCP_8MHZ);
        mcp2515_setNormalMode(&mcp2515);

        if (mcp2515_reset(&mcp2515) != ERROR_OK)
        {
            printf("mcp2515_reset FAILED\r\n");
            printf("retry %d times\r\n", i + 1);
            continue;
        }
        if (mcp2515_setBitrate(&mcp2515, CAN_125KBPS, MCP_8MHZ) != ERROR_OK)
        {
            printf("mcp2515_setBitrate FAILED\r\n");
            printf("retry %d times\r\n", i + 1);
            continue;
        }
        if (mcp2515_setNormalMode(&mcp2515) != ERROR_OK)
        {
            printf("mcp2515_setNormalMode FAILED\r\n");
            printf("retry %d times\r\n", i + 1);
            continue;
        }
        printf("All SUSSES start Send\r\n");
        mcp2515_delay_ms(100);
        break;
    }

    while (1)
    {
        mcp2515_sendMessage(&mcp2515, &tx_frame_1);
        mcp2515_delay_ms(1000);

        mcp2515_sendMessage(&mcp2515, &tx_frame_2);
        mcp2515_delay_ms(1000);
    }
    return 0;
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

void mcp2515_select(void)
{
    gpio_clear(SPI_PORT, SPI_CS_PIN); /* CS pin output low to select. */
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

/* INT pin interrupt handler. */
void exti9_5_isr(void)
{
    can_frame_t rx_frame;
    uint8_t irq = mcp2515_getInterrupts(&mcp2515);
    if (irq & CANINTF_RX0IF)
    {
        if (mcp2515_readMessage_RXBn(&mcp2515, RXB0, &rx_frame) == ERROR_OK)
        {
            rx_frame.can_id = 0x0F0;
            mcp2515_sendMessage(&mcp2515, &rx_frame);
        }
    }
    if (irq & CANINTF_RX1IF)
    {
        if (mcp2515_readMessage_RXBn(&mcp2515, RXB1, &rx_frame) == ERROR_OK)
        {
            rx_frame.can_id = 0x0F1;
            mcp2515_sendMessage(&mcp2515, &rx_frame);
        }
    }

    exti_reset_request(INT_EXTI);
}

/* For printf(). */
int _write(int file, char *ptr, int len)
{
    int i;

    if (file == 1)
    {
        for (i = 0; i < len; i++)
        {
            usart_send_blocking(CHOSEN_USART_NUM, ptr[i]);
        }
        return i;
    }

    errno = EIO;
    return -1;
}