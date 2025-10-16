# CSCI 4900/6900 mmWave Radar Project Demo

This repository contains a simple demo for **TI IWR1843BOOST mmWave radar board**, using the **Out-of-Box (OOB) demo firmware** to collect radar data and estimate breathing rate.

---

## Hardware
- **Board**: Texas Instruments IWR1843BOOST
- **Connections**:
  - Micro-USB cable for **power + UART** (two COM ports exposed: CLI + DATA)
  - Optional: DCA1000EVM for raw data streaming (not required for this demo)

---

## Software
- **TI mmWave SDK (v3.x)**: provides the OOB demo firmware
- **mmWave Demo Visualizer (v3.x)**: used to generate `.cfg` configuration files
- **Python 3.8+**
- Required Python libraries:
  ```bash
  pip install pyserial numpy matplotlib scipy
