# Aurora RS485 LoRa Bridge

Bridge an ABB/Fimer Aurora photovoltaic inverter over RS485 to LoRaWAN using a Heltec Wireless Stick Lite V3 (ESP32) and the ABB Aurora Solar Inverter Arduino library. The repository contains two sketches: one for field deployment that periodically uplinks inverter telemetry via LoRaWAN, and another for direct serial data capture while validating inverter connectivity.

## Features
- Reads power, voltage, temperature, energy totals, device state, alarm codes, and identification information from Aurora inverters over RS485.
- Packs measurements into a compact LoRaWAN payload and triggers confirmed uplinks when alarms are detected.
- Supports both OTAA and ABP activation with configurable region, class, and duty-cycle.
- Simple serial-only sketch for debugging inverter communication without LoRaWAN.

## Repository structure
- `Aurora_Simulation_Field/`: LoRaWAN-enabled sketch that polls the inverter and transmits telemetry.
- `Inverter_DataCapture/`: Standalone sketch that prints inverter readings over USB serial for bench testing.
- `lib/ABB_Aurora_Solar_Inverter_Library/`: Bundled Aurora inverter Arduino library used by both sketches.

## Hardware requirements
- **Heltec Wireless Stick Lite V3 (ESP32)** or compatible board with RS485 transceiver support.
- **RS485 transceiver** (e.g., MAX3485) wired to ESP32 UART2. The field sketch expects TX on GPIO 17, RX on GPIO 18, and the simulation sketch uses TX on 17, RX on 16 with a control pin on GPIO 21.
- **ABB/Fimer Aurora inverter** configured with a device address (default `2`).

## Getting started
1. **Install the library**: Copy `lib/ABB_Aurora_Solar_Inverter_Library` into your Arduino `libraries` folder.
2. **Open the sketch**: Choose either `Aurora_Simulation_Field/Aurora_Simulation_Field.ino` for LoRaWAN telemetry or `Inverter_DataCapture/Inverter_DataCapture/Inverter_DataCapture.ino` for serial capture.
3. **Configure parameters**:
   - Confirm `INVERTER_ADDRESS` matches the inverter’s RS485 address.
   - Adjust RS485 pin assignments to match your wiring (`RS485_CONTROL_PIN`, `RS485_RX_PIN`, `RS485_TX_PIN`).
   - Set LoRaWAN credentials (`devEui`, `appEui`, `appKey` for OTAA or `nwkSKey`, `appSKey`, `devAddr` for ABP) and region/class in the field sketch.
   - Optionally change `appTxDutyCycle` (default 30 seconds) and `UNIFIED_DATA_FPORT` (default 10).
4. **Select the board and port** in the Arduino IDE (`Heltec Wireless Stick Lite V3`) and upload the sketch.
5. **Monitor output** via the serial console at 115200 baud. The field sketch logs each measurement and the prepared LoRaWAN payload; the capture sketch prints a full set of inverter registers every 5 seconds.

## LoRaWAN payload layout
The field sketch packs measurements into a 38-byte payload (little-endian floats where applicable) on FPort 10:

| Offset | Length | Field |
| ------ | ------ | ----- |
| 0      | 4      | Grid power (W) |
| 4      | 4      | Grid voltage (V) |
| 8      | 4      | PV voltage (V) |
| 12     | 4      | Inverter temperature (°C) |
| 16     | 4      | Daily energy (Wh, unsigned 32-bit) |
| 20     | 1      | Global state code |
| 21     | 1      | Alarm state code |
| 22     | 4      | Last four alarms (newest first) |
| 26     | 6      | Part number (ASCII) |
| 32     | 6      | Serial number (ASCII) |

If `alarmState` is non-zero, the sketch switches to confirmed uplinks for the current transmission.

## RS485 wiring notes
- UART2 runs at `19200 8N1` for inverter communication.
- The field sketch uses an auto-flow-control module (no direction pin). The simulation sketch drives `RS485_CONTROL_PIN` directly when using a transceiver that requires manual direction control.

## License
The bundled Aurora inverter library includes its own `LICENSE.md`; project-specific code follows the same licensing terms unless otherwise noted.
