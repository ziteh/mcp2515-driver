# MCP2515 Driver

This is a platform independent MCP2515 CAN interface library.

> This library was modified under the MIT license from [autowp/arduino-mcp2515](https://github.com/autowp/arduino-mcp2515).

## Initialization

To create connection with MCP2515 provide SPI control functions, baudrate and mode.

The available modes are listed as follows:
```C++
mcp2515.setNormalMode();
mcp2515.setLoopbackMode();
mcp2515.setListenOnlyMode();
```

The available baudrates are listed as follows:
```C++
enum CAN_SPEED {
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
};
```

Example of initialization:
```C++
void spiSelect(void)
{
  gpio_clear(GPIO_CS_PORT, GPIO_CS_PIN); /* CS pin low. */
}

void spiDeselect(void)
{
  while (!(SPI_SR(SPI1) & SPI_SR_TXE));
  while ((SPI_SR(SPI1) & SPI_SR_BSY));
  gpio_set(GPIO_CS_PORT, GPIO_CS_PIN); /* CS pin high. */
}

uint8_t spiTransfer(uint8_t data)
{
  uint16_t rec = spi_xfer(SPI1, data); /* SPI data write and read. */
  return rec & 0xFF;
}

void mcp2515Init(void)
{
  MCP2515 mcp2515(&spiSelect, &spiDeselect, &spiTransfer, &delay);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
}
```

You can also set oscillator frequency for module when setting bitrate:
```C++
mcp2515.setBitrate(CAN_125KBPS, MCP_16MHZ);
```

The available clock speeds are listed as follows:
```C++
enum CAN_CLOCK {
    MCP_20MHZ,
    MCP_16MHZ,
    MCP_8MHZ
};
```

Default value is `MCP_8MHZ`

*Note*: To transfer data on high speed of CAN interface via UART dont forget to update UART baudrate as necessary.

## Frame data format

Library uses Linux-like structure to store can frames;

```C++
struct can_frame {
    uint32_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    uint8_t  can_dlc;
    uint8_t  data[8];
};
```

For additional information see [SocketCAN](https://www.kernel.org/doc/Documentation/networking/can.txt)

## Send Data

```C++
MCP2515::ERROR sendMessage(const MCP2515::TXBn txbn, const struct can_frame *frame);
MCP2515::ERROR sendMessage(const struct can_frame *frame);
```

This is a function to send data onto the bus.

For example, In the 'send' example, we have:

```C++
struct can_frame frame;
frame.can_id = 0x000;
frame.can_dlc = 4;
frame.data[0] = 0xFF;
frame.data[1] = 0xFF;
frame.data[2] = 0xFF;
frame.data[3] = 0xFF;

/* send out the message to the bus and
tell other devices this is a standard frame from 0x00. */
mcp2515.sendMessage(&frame);
```

```C++
struct can_frame frame;
frame.can_id = 0x12345678 | CAN_EFF_FLAG;
frame.can_dlc = 2;
frame.data[0] = 0xFF;
frame.data[1] = 0xFF;

/* send out the message to the bus using second TX buffer and
tell other devices this is a extended frame from 0x12345678. */
mcp2515.sendMessage(MCP2515::TXB1, &frame);
```

## Receive Data

The following function is used to receive data on the 'receive' node:

```C++
MCP2515::ERROR readMessage(const MCP2515::RXBn rxbn, struct can_frame *frame);
MCP2515::ERROR readMessage(struct can_frame *frame);
```

In conditions that masks and filters have been set. This function can only get frames that meet the requirements of masks and filters.

You can choise one of two method to receive: interrupt-based and polling

Example of poll read

```C++
struct can_frame frame;

void loop() {
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        // frame contains received message
    }
}
```

Example of interrupt based read

```C++
volatile bool interrupt = false;
struct can_frame frame;

void irqHandler() {
    interrupt = true;
}

void setup() {
    ...
    attachInterrupt(0, irqHandler, FALLING);
}

void loop() {
    if (interrupt) {
        interrupt = false;

        uint8_t irq = mcp2515.getInterrupts();

        if (irq & MCP2515::CANINTF_RX0IF) {
            if (mcp2515.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
                // frame contains received from RXB0 message
            }
        }

        if (irq & MCP2515::CANINTF_RX1IF) {
            if (mcp2515.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK) {
                // frame contains received from RXB1 message
            }
        }
    }
}
```


## Set Receive Mask and Filter

There are 2 receive mask registers and 5 filter registers on the controller chip that guarantee you get data from the target device. They are useful especially in a large network consisting of numerous nodes.

We provide two functions for you to utilize these mask and filter registers. They are:

```C++
MCP2515::ERROR setFilterMask(const MASK mask, const bool ext, const uint32_t ulData)
MCP2515::ERROR setFilter(const RXF num, const bool ext, const uint32_t ulData)
```

**MASK mask** represents one of two mask `MCP2515::MASK0` or `MCP2515::MASK1`

**RXF num** represents one of six acceptance filters registers from `MCP2515::RXF0` to `MCP2515::RXF5`

**ext** represents the status of the frame. `false` means it's a mask or filter for a standard frame. `true` means it's for a extended frame.

**ulData** represents the content of the mask of filter.

----

This software is written by loovee ([luweicong@seeed.cc](luweicong@seeed.cc "luweicong@seeed.cc")) for seeed studio,  
Updated by Dmitry ([https://github.com/autowp](https://github.com/autowp "https://github.com/autowp")),  
Modified by ZiTe ([https://github.com/ziteh](https://github.com/ziteh)),  
and is licensed under [The MIT License](http://opensource.org/licenses/mit-license.php). Check [`LICENSE`](LICENSE) for more information.  

Contributing to this software is warmly welcomed. You can do this basically by  
[forking](https://help.github.com/articles/fork-a-repo), committing modifications and then [pulling requests](https://help.github.com/articles/using-pull-requests) (follow the links above  
for operating guide). Adding change log and your contact into file header is encouraged.  
Thanks for your contribution.

Seeed Studio is an open hardware facilitation company based in Shenzhen, China.  
Benefiting from local manufacture power and convenient global logistic system,  
we integrate resources to serve new era of innovation. Seeed also works with  
global distributors and partners to push open hardware movement.  
