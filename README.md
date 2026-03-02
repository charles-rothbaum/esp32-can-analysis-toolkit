# ESP32-S3 CAN Sniffer for SavvyCan

This project turns an ESP32-S3 into a CAN bus sniffer that streams frames to [SavvyCAN](https://www.savvycan.com/) using the GVRET serial protocol.

## What It Does

- Reads CAN traffic from the ESP32 TWAI peripheral.
- Streams frames over UART at `1,000,000` baud.
- Supports SavvyCAN GVRET commands needed for live sniffing.
- Starts in listen-only mode by default for safer bus monitoring.

## Hardware

- ESP32-S3 dev board
- CAN transceiver (I am using the Adafruit CAN PAL)
- Access to target CANH/CANL bus

## Wiring

Default pins in code:

- `TWAI_TX` -> `GPIO5`
- `TWAI_RX` -> `GPIO4`
- `GVRET UART` -> `UART0` at `1,000,000` baud

Transceiver wiring:

- ESP32 `TWAI_TX` -> transceiver `TXD`
- ESP32 `TWAI_RX` -> transceiver `RXD`
- transceiver `CANH/CANL` -> target CAN bus

## Software Requirements

- ESP-IDF 5.x
- SavvyCAN

## Build and Flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/tty.usbmodemXXXX flash
idf.py -p /dev/tty.usbmodemXXXX monitor
```

Replace `/dev/tty.usbmodemXXXX` with your actual serial device.

## SavvyCAN Setup

1. Open SavvyCAN.
2. Choose the GVRET/Serial connection.
3. Select the ESP32 serial port.
4. Set baud rate to `1,000,000`.
5. Connect and start capture.

## Configuration

Tune these values in source if needed:

- CAN bitrate (default `500000`) but try 250000 or 125000 if unsuccesful
- TX/RX GPIO pins
- UART port (`UART0` vs `UART1`)
- Listen-only mode