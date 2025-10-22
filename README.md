# CSCI 4900/6900 mmWave Radar Project Demo

This repository contains a simple demo for **TI IWR1843BOOST mmWave radar board**, using the **Out-of-Box (OOB) demo firmware** to collect radar data and estimate breathing rate.

---

## Hardware
- **Board**: Texas Instruments IWR1843BOOST
- **Connections**:
  - Micro-USB cable for **power + UART** (two COM ports exposed: CLI + DATA)
  - Optional: DCA1000EVM for raw data streaming (not required for this demo)

xWR1843 Chip: TX → Mixer → Baseband → ADC → DSP → MSS → UART Output

<img width="741" height="396" alt="image" src="https://github.com/user-attachments/assets/7927b909-4dfb-45c3-8d8b-30408cf9608e" />

### Antenna:

<img width="435" height="391" alt="image" src="https://github.com/user-attachments/assets/827a6700-8375-400c-a32b-6cab2d359d73" />

### Sense-on-Power (SOP) Jumpers

Three different modes based on the state of the SOP lines.

1. Functional Mode: **Normal radar sensing**
2. Flashing Mode: Used for **firmware programming** via UART
3. Debug Mode: Internal signal path test (No external antenna or reflection needed)

<img width="640" height="144" alt="image" src="https://github.com/user-attachments/assets/c9f25344-0bfa-4a22-9559-458e0942a67b" />

### Configuration

In linear FMCW radars, the transmit (TX) signal is a single tone with its frequency changing linearly with time. This sweep in frequency is commonly referred to as a “chirp”. A set of these chirps form a “Frame” and this can be used as the observation window for the radar processing. The various parameters of the chirp ramp (like frequency slope, sweep bandwidth, and so forth) impact the system performance. [guide](https://www.ti.com/lit/an/swra553a/swra553a.pdf?ts=1761085003689)

<img width="1031" height="653" alt="image" src="https://github.com/user-attachments/assets/a5faaa41-dbc4-4c9a-a10c-04e62f8a49da" />

A .cfg (configuration) file is a plain text file that tells a TI mmWave radar board (like xWR1843, xWR6843, IWR1443, etc.) how to generate chirps, frames, and process data.

When you use mmWave Studio or a Python/CLI or MATLAB script with TI’s mmWave SDK, this file is sent line-by-line over UART to configure:

```bash
sensorStop                 % stop
flushCfg                   % remove previous cfg
dfeDataOutputMode 1        % 1 = frame-based mode
channelCfg 15 3 0          % TX/RX antenna config
adcCfg 2 1                 % ADC bits, output format
adcbufCfg 0 1 0 1          % ADC buffer settings
% profileCfg <profileId> <startFreq_GHz> <idleTime_us> <adcStartTime_us> <rampEndTime_us> <txPower> <txPhaseShift> <freqSlope_MHz/us> <numAdcSamples> <sampleRate_ksps> <hpfCornerFreq1> <hpfCornerFreq2> <rxGain_dB>
profileCfg 0 77 7 3 48 0 0 100 1 64 2500 0 0 30
% chirpCfg <chirpStartIdx> <chirpEndIdx> <profileId> <startFreqVar> <freqSlopeVar> <idleTimeVar> <adcStartVar> <txEnable>
chirpCfg 0 0 0 0 0 0 0 1
chirpCfg 1 1 0 0 0 0 0 2
% frameCfg <chirpStartIdx> <chirpEndIdx> <numLoops> <numFrames> <framePeriodicity_ms> <triggerSelect> <triggerDelay>
frameCfg 0 1 128 0 100 1 0
lowPower 0 1
guiMonitor 1 1 1 1 0
cfarCfg 0 2 8 4 3 0 2560
multiObjBeamForming 1 0.5
clutterRemoval 1
calibDcRangeSig 0 -5 8 256
sensorStart
```

---

## Software
- **TI mmWave SDK (v3.x)**: provides the OOB demo firmware
- **mmWave Demo Visualizer (v3.x)**: used to generate `.cfg` configuration files
- **Python 3.8+**
- Required Python libraries:
  ```bash
  pip install pyserial numpy matplotlib scipy
