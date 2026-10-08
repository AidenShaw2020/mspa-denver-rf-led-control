# MSpa Denver RF LED control

ESPHome support for the 433 MHz RF remote supplied with an MSpa Denver LED strip. An ESP32 and CC1101 can send all 17 remote commands from Home Assistant and listen for presses on the original remote. An Arduino diagnostic sketch is included for capture, frequency checks, and manual transmissions.

**Status:** The ESPHome control has been tested with the LED strip, and the user confirmed that the commands work. The radio does not report the strip's current state. Home Assistant therefore gets button entities rather than assumed on/off, color, or brightness values.

## Hardware

Use an ESP32 Dev Module and a CC1101 module with a 26 MHz crystal. The CC1101 must use **3.3 V power and logic**.

| CC1101 pin | ESP32 pin |
| --- | --- |
| GND | GND |
| VCC | 3V3 |
| GDO0 | GPIO17 |
| CSN | GPIO5 |
| SCK | GPIO18 |
| MOSI | GPIO23 |
| MISO / GDO1 | GPIO19 |
| GDO2 | GPIO16 |

Reception uses GDO1 on the shared MISO line. GDO0 and GDO2 remain connected for radio diagnostics. Check your module's pin labels before applying power.

## ESPHome installation

1. Copy [`esphome/mspa-denver-led-rf.yaml`](esphome/mspa-denver-led-rf.yaml) to `/config/esphome/mspa-denver-led-rf.yaml`.
2. Copy the entire [`esphome/components/mspa_led_rf/`](esphome/components/mspa_led_rf/) directory to `/config/esphome/components/mspa_led_rf/`.
3. Add your own Wi-Fi credentials to `/config/esphome/secrets.yaml` as `wifi_ssid` and `wifi_password`. You may add ESPHome API encryption and OTA authentication to the YAML for your installation.
4. In ESPHome, install `mspa-denver-led-rf.yaml` on the ESP32. The configuration uses the Arduino framework and was validated against ESPHome 2026.9.0.
5. In Home Assistant, press **White** or **Power** and confirm the LED strip responds. Press the corresponding original remote button and check the **Remote button** event and **Last received button** text sensor.

The configuration creates individual button entities for Power, Mode +/−, Speed +/−, Demo, Color +/−, Brightness +/−, White, Red, Green, Blue, Yellow, Cyan, and Pink. **Last transmitted button** records what the ESP32 queued for RF transmission; it is not feedback from the strip.

No RF command is sent automatically at boot. Each Home Assistant button press sends four approximately 51.1 ms OOK periods, then returns the CC1101 to receive mode.

## Frequency calibration

The original remote was measured around **433.970–433.973 MHz**. On the tested CC1101, setting `tx_frequency: 434.0254` produced approximately **433.9722 MHz on air**. This programmed-to-air offset is specific to the tested module. If your strip does not respond, compare the ESP32 and original remote carriers with an SDR, then adjust `tx_frequency` in the YAML. The default `rx_frequency` is 433.973 MHz.

The receiver uses asynchronous OOK, approximately 101.6 kHz bandwidth, and GDO1 edge timing. The transmitted waveform uses 40 kbaud FIFO OOK with a low-power PA setting (approximately −10 dBm on a typical 433 MHz CC1101 module).

## Arduino diagnostic sketch

Open [`arduino/MspaLedSniffer/MspaLedSniffer.ino`](arduino/MspaLedSniffer/MspaLedSniffer.ino) in Arduino IDE, select **ESP32 Dev Module**, and open Serial Monitor at 115200 baud with newline termination. It uses only the ESP32 Arduino core and SPI library.

Useful commands:

| Command | Purpose |
| --- | --- |
| `status`, `pins`, `rssi` | Inspect radio settings, wiring, and signal level |
| `start 15000`, `stop`, `dump` | Capture and print raw HIGH/LOW edge timings |
| `source gdo1` | Select the tested asynchronous receive output |
| `freq 433.973` | Set the programmed RX frequency in MHz |
| `txfreq 434.0254` | Set the programmed TX frequency in MHz |
| `send white` | Manually send one mapped command |
| `send all` | Send all 17 commands, one second apart |

The Arduino sketch and ESPHome firmware are alternatives for the same board; flash the ESPHome configuration again after using the sketch.

## Protocol notes

One approximately 51.1 ms period contains a short sync mark, a 12.6 ms delay to the first data mark, then 24 marks spaced 1.6 ms apart. A short mark is around 0.475 ms and a long mark around 1.275 ms. The first 19 data marks are fixed; the final five encode the button command (`S=0`, `L=1`). A captured 26th mark belongs to the **next** period's sync.

This is an independent community project and is not affiliated with MSpa. Licensed under [MIT](LICENSE).
