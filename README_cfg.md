# 📘 IWR1843 OOB Demo Configuration Cheat Sheet

The `.cfg` file is a script sent over the **CLI UART** to configure both radar hardware (RF/ADC) and the signal processing chain (FFT, CFAR, AoA).  

---

## 1. Control Commands
| Command | Description |
|---------|-------------|
| `sensorStop` | Stops the radar (required before sending new config). |
| `flushCfg` | Clears any previous configuration. |
| `dfeDataOutputMode <mode>` | Output mode: `1` = processed TLVs, `2` = raw ADC (DCA1000). |

---

## 2. Channel and ADC
| Command | Description |
|---------|-------------|
| `channelCfg <rxMask> <txMask> <cascading>` | Enable Rx/Tx antennas. `15` = 4 Rx on, `7` = 3 Tx on. |
| `adcCfg <bits> <format>` | ADC bits and format. `2 1` = 16-bit complex I/Q. |
| `adcbufCfg ...` | Configures ADC buffer storage (IQ swap, interleave). |

---

## 3. Profile and Chirps
| Command | Description |
|---------|-------------|
| `profileCfg <id> <startFreq> <idleTime> <adcStartTime> <rampEndTime> ...` | Defines chirp waveform (start freq, slope, samples, sampling rate, Rx gain). |
| `chirpCfg <startIdx> <endIdx> <profileId> ... <txEnable>` | Assigns profile to Tx antenna(s). Used for TDM-MIMO. |

---

## 4. Frame Timing
| Command | Description |
|---------|-------------|
| `frameCfg <chirpStart> <chirpEnd> <numLoops> <numFrames> <periodicity> <triggerSel> <delay>` | Frame structure. `numFrames=0` = infinite. `periodicity` = frame rate (ms). |

---

## 5. Processing Chain
| Command | Description |
|---------|-------------|
| `lowPower <subFrameIdx> <enabled>` | Enables low-power mode. |
| `guiMonitor <sf> <detPoints> <rangeProf> <noiseProf> <rangeAzHeat> <rangeDopHeat> <stats>` | Selects which TLVs to output. |
| `cfarCfg <sf> <procDir> <avgMode> <winLen> <guardLen> <noiseDiv> <cyclic> <scale> ...` | CFAR detection config. `procDir=0`=range, `1`=Doppler. |
| `peakGrouping ...` | Groups nearby peaks into single detections. |
| `multiObjBeamForming <sf> <enabled> <threshold>` | Separates multiple objects in AoA. |
| `clutterRemoval <sf> <enabled>` | Removes static clutter (important for breathing/vitals). |
| `calibDcRangeSig <sf> <enabled> <negBin> <posBin> <numAvg>` | Calibrates DC offset in range bins. |
| `extendedMaxVelocity <sf> <enabled>` | Doubles unambiguous velocity using inter-frame techniques. |

---

## 6. Calibration & Start
| Command | Description |
|---------|-------------|
| `compRangeBiasAndRxChanPhase ...` | Apply range bias & Rx phase compensation. |
| `measureRangeBiasAndRxChanPhase <en> <targetDist> <searchWin>` | Auto measure bias with known reflector. |
| `sensorStart` | Starts the radar. |

---

## 📊 Performance Relations
- **Range resolution (ΔR)**  
  \[
  \Delta R = \frac{c}{2 \cdot BW}, \quad BW = \text{freqSlopeConst} \times \text{rampEndTime}
  \]

- **Max unambiguous range**  
  \[
  R_{\max} = \frac{c}{2 f_s} \cdot N
  \]

- **Velocity resolution**  
  \[
  \Delta v = \frac{\lambda}{2 \cdot T_\text{frame} \cdot N_\text{loops}}
  \]

- **Max unambiguous velocity**  
  Depends on chirp repetition frequency (PRF).  

---

✅ **In short:**  
- `.cfg` controls **antennas, chirps, frame rate, FFT/CFAR, and output TLVs**.  
- You tune **bandwidth → range resolution**, **numLoops → velocity resolution**, and **antenna masks → angle estimation**.  
