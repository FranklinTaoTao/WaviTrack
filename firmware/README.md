# WaviTrack firmware

`v3/data_collection` is the six-node ESP32-C6 application: DW3000 ranging and CIR, BNO085 IMU, and UDP packets. The `v2` and `v3` folders also contain board-specific UWB, node-ID, and pairwise SS-TWR examples. `libraries/ESP32-DW3000` is the modified driver; `python/` contains serial CIR viewers and the UDP-to-NPZ receiver.

## Setup

Install the ESP32 Arduino board package and the included DW3000 library. For V3 IMU sketches, install **SparkFun BNO08x Arduino Library** from Arduino IDE's Library Manager. The Python tools use `numpy`, `matplotlib`, and `pyserial`.

For two-board ranging, flash `pairwise_initiator` and `pairwise_responder` from the matching board revision. The initiator prints distance over serial at 115200 baud.

For six-node V3 collection, edit `v3/node_idx/node_idx.ino` to write a distinct ID **0–5** on each board, then flash `v3/data_collection/data_collection.ino`. Set `WIFI_SSID`, `WIFI_PSK`, and `UDP_SERVER_IP` in that sketch to your network and receiver. Its default configuration waits for Wi-Fi before ranging. Run `python/udp_capture_to_npz.py` on the receiver; it listens on UDP ports **20000–20005** and saves an NPZ capture.

## Reproduction tips

- The radio examples use **20 MHz SPI after initialization**; startup uses a slower clock.
- **40 MHz SPI is outside the DW3000 specification.** It may be tried only if your hardware supports it and you validate its operation; 20 MHz is the release baseline.
- Antenna delay affects absolute ranging accuracy. Calibrate it before comparing measured distance with a known separation.
