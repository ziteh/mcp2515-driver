/**
 * @file mcp2515.h
 * @brief Platform independent MCP2515 CAN interface library header file.
 * @author Seeed Technology Inc.,
 *         Dmitry,
 *         ZiTe (honmonoh@gmail.com)
 * @copyright MIT License.
 * @remark This library was modified from 'autowp/arduino-mcp2515' (https://github.com/autowp/arduino-mcp2515).
 */

#ifndef MCP2515_H_
#define MCP2515_H_

#include "can.h"

typedef enum
{
  MCP_20MHZ,
  MCP_16MHZ,
  MCP_8MHZ
} CAN_CLOCK;

typedef enum
{
  CAN_5KBPS,
  CAN_10KBPS,
  CAN_20KBPS,
  CAN_31K25BPS,
  CAN_33KBPS,
  CAN_40KBPS,
  CAN_50KBPS,
  CAN_80KBPS,
  CAN_83K3BPS,
  CAN_95KBPS,
  CAN_100KBPS,
  CAN_125KBPS,
  CAN_200KBPS,
  CAN_250KBPS,
  CAN_500KBPS,
  CAN_1000KBPS
} CAN_SPEED;

typedef enum
{
  CLKOUT_DISABLE = -1,
  CLKOUT_DIV1 = 0x0,
  CLKOUT_DIV2 = 0x1,
  CLKOUT_DIV4 = 0x2,
  CLKOUT_DIV8 = 0x3,
} CAN_CLKOUT;

typedef enum
{
  ERROR_OK = 0,
  ERROR_FAIL = 1,
  ERROR_ALLTXBUSY = 2,
  ERROR_FAILINIT = 3,
  ERROR_FAILTX = 4,
  ERROR_NOMSG = 5
} ERROR;

typedef enum
{
  MASK0,
  MASK1
} MASK;

typedef enum
{
  RXF0 = 0,
  RXF1 = 1,
  RXF2 = 2,
  RXF3 = 3,
  RXF4 = 4,
  RXF5 = 5
} RXF;

typedef enum
{
  RXB0 = 0,
  RXB1 = 1
} RXBn;

typedef enum
{
  TXB0 = 0,
  TXB1 = 1,
  TXB2 = 2
} TXBn;

typedef enum
{
  CANINTF_RX0IF = 0x01,
  CANINTF_RX1IF = 0x02,
  CANINTF_TX0IF = 0x04,
  CANINTF_TX1IF = 0x08,
  CANINTF_TX2IF = 0x10,
  CANINTF_ERRIF = 0x20,
  CANINTF_WAKIF = 0x40,
  CANINTF_MERRF = 0x80
} CANINTF;

typedef enum
{
  EFLG_RX1OVR = (1 << 7),
  EFLG_RX0OVR = (1 << 6),
  EFLG_TXBO = (1 << 5),
  EFLG_TXEP = (1 << 4),
  EFLG_RXEP = (1 << 3),
  EFLG_TXWAR = (1 << 2),
  EFLG_RXWAR = (1 << 1),
  EFLG_EWARN = (1 << 0)
} EFLG;

typedef void (*mcp2515_spi_select_t)(void);
typedef void (*mcp2515_spi_deselect_t)(void);
typedef uint8_t (*mcp2515_spi_transfer_t)(uint8_t data);
typedef void (*mcp2515_delay_ms_t)(uint32_t ms);

typedef struct
{
  mcp2515_spi_select_t spi_select;
  mcp2515_spi_deselect_t spi_deselect;
  mcp2515_spi_transfer_t spi_transfer;
  mcp2515_delay_ms_t delay;
} mcp2515_handle_t;

int8_t mcp2515_make_handle(
    mcp2515_spi_select_t spi_select_func,
    mcp2515_spi_deselect_t spi_deselect_fun,
    mcp2515_spi_transfer_t spi_transfer_fun,
    mcp2515_delay_ms_t delay_func,
    mcp2515_handle_t *mcp2515_handle);

ERROR mcp2515_reset(const mcp2515_handle_t *mcp2515_handle);
ERROR mcp2515_setConfigMode(const mcp2515_handle_t *mcp2515_handle);
ERROR mcp2515_setListenOnlyMode(const mcp2515_handle_t *mcp2515_handle);
ERROR mcp2515_setSleepMode(const mcp2515_handle_t *mcp2515_handle);
ERROR mcp2515_setLoopbackMode(const mcp2515_handle_t *mcp2515_handle);
ERROR mcp2515_setNormalMode(const mcp2515_handle_t *mcp2515_handle);
ERROR mcp2515_setClkOut(const mcp2515_handle_t *mcp2515_handle, const CAN_CLKOUT divisor);
ERROR mcp2515_setBitrate(const mcp2515_handle_t *mcp2515_handle, const CAN_SPEED canSpeed, const CAN_CLOCK canClock);
ERROR mcp2515_setFilterMask(const mcp2515_handle_t *mcp2515_handle, const MASK num, const bool ext, const uint32_t ulData);
ERROR mcp2515_setFilter(const mcp2515_handle_t *mcp2515_handle, const RXF num, const bool ext, const uint32_t ulData);
ERROR mcp2515_sendMessage_TXBn(const mcp2515_handle_t *mcp2515_handle, const TXBn txbn, const can_frame_t *frame);
ERROR mcp2515_sendMessage(const mcp2515_handle_t *mcp2515_handle, const can_frame_t *frame);
ERROR mcp2515_readMessage_RXBn(const mcp2515_handle_t *mcp2515_handle, const RXBn rxbn, can_frame_t *frame);
ERROR mcp2515_readMessage(const mcp2515_handle_t *mcp2515_handle, can_frame_t *frame);

bool mcp2515_checkReceive(const mcp2515_handle_t *mcp2515_handle);
bool mcp2515_checkError(const mcp2515_handle_t *mcp2515_handle);

uint8_t mcp2515_getErrorFlags(const mcp2515_handle_t *mcp2515_handle);
void mcp2515_clearRXnOVRFlags(const mcp2515_handle_t *mcp2515_handle);

uint8_t mcp2515_getInterrupts(const mcp2515_handle_t *mcp2515_handle);
uint8_t mcp2515_getInterruptMask(const mcp2515_handle_t *mcp2515_handle);
void mcp2515_clearInterrupts(const mcp2515_handle_t *mcp2515_handle);
void mcp2515_clearTXInterrupts(const mcp2515_handle_t *mcp2515_handle);

uint8_t mcp2515_getStatus(const mcp2515_handle_t *mcp2515_handle);

void mcp2515_clearRXnOVR(const mcp2515_handle_t *mcp2515_handle);
void mcp2515_clearMERR(const mcp2515_handle_t *mcp2515_handle);
void mcp2515_clearERRIF(const mcp2515_handle_t *mcp2515_handle);

uint8_t mcp2515_errorCountRX(const mcp2515_handle_t *mcp2515_handle);
uint8_t mcp2515_errorCountTX(const mcp2515_handle_t *mcp2515_handle);

#endif /* MCP2515_H_ */
