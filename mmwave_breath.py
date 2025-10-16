#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# IWR1843 continuous breathing monitor over UART (SDK Out-of-Box Demo)
# - Sends cfg over CLI UART
# - Parses DATA UART TLVs (range profile)
# - Tracks dominant static range bin and continuously estimates BPM
# - Live plots the dominant-bin amplitude (last `--window` seconds)

import os
import sys
import time
import struct
from collections import deque
from pathlib import Path

import numpy as np
import serial
import matplotlib.pyplot as plt

# -------------------- Optional SciPy band-pass --------------------
try:
    from scipy.signal import butter, filtfilt
    HAVE_SCIPY = True
except Exception:
    HAVE_SCIPY = False

# -------------------- Try to use parseFrame if available ----------
USE_PARSEFRAME = False
UART_MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'  # default SDK magic
parseStandardFrame = None
try:
    from parseFrame import parseStandardFrame as _psf, UART_MAGIC_WORD as _MW
    parseStandardFrame = _psf
    UART_MAGIC_WORD = _MW
    USE_PARSEFRAME = True
    print("[INFO] Using parseFrame.py for frame parsing.")
except Exception:
    print("[INFO] parseFrame.py not found; using internal range-profile parser.")

# -------------------- CLI helpers --------------------
def read_cli_until_quiet(cli, quiet_ms=250):
    """Drain CLI replies until silence for quiet_ms."""
    old_to = cli.timeout
    cli.timeout = 0.05
    out = []
    last = time.time()
    while True:
        line = cli.readline()
        if line:
            try:
                out.append(line.decode(errors='ignore').rstrip())
            except Exception:
                out.append(repr(line))
            last = time.time()
        if (time.time() - last) * 1000.0 > quiet_ms:
            break
    cli.timeout = old_to
    return out

def send_cfg(cli_ser, cfg_path_or_lines):
    """Send .cfg lines to CLI UART. Accepts a path or list of strings. Prints echoes; raises on error."""
    if isinstance(cfg_path_or_lines, (str, os.PathLike)):
        text = Path(cfg_path_or_lines).read_text(encoding="utf-8", errors="ignore")
        raw_lines = text.splitlines()
    else:
        raw_lines = list(cfg_path_or_lines)

    lines = []
    for ln in raw_lines:
        s = str(ln).strip()
        if not s or s.startswith('%'):
            continue
        if not s.endswith('\n'):
            s += '\n'
        lines.append(s)

    # ensure stop/start
    if not any(l.lower().startswith("sensorstop") for l in lines):
        lines.insert(0, "sensorStop\n")
    if not any(l.lower().startswith("sensorstart") for l in lines):
        lines.append("sensorStart\n")

    print(f"[CFG] Sending {len(lines)} lines...")
    for ln in lines:
        time.sleep(0.03)
        if cli_ser.baudrate == 1250000:
            for ch in ln:
                cli_ser.write(ch.encode()); time.sleep(0.001)
        else:
            cli_ser.write(ln.encode())
        resp = read_cli_until_quiet(cli_ser, quiet_ms=200)
        print(f"[CLI] {ln.strip()} -> {resp}")
        if any(("Error" in r) or ("error" in r) for r in resp):
            raise RuntimeError(f"CLI rejected line: {ln.strip()} | resp={resp}")

    time.sleep(0.05)
    cli_ser.reset_input_buffer()

# -------------------- DATA UART frame reader --------------------
def read_one_frame(ser):
    """Read one complete SDK OOB frame from DATA UART. Returns bytes or None."""
    # seek magic
    idx = 0
    frame = bytearray()
    while True:
        b = ser.read(1)
        if len(b) < 1:  # timeout
            return None
        if b[0] == UART_MAGIC_WORD[idx]:
            frame.append(b[0]); idx += 1
            if idx == 8:
                break
        else:
            if idx != 0 and b[0] == UART_MAGIC_WORD[0]:
                idx = 1
                frame = bytearray([UART_MAGIC_WORD[0]])
            else:
                idx = 0
                frame = bytearray()

    header = ser.read(32)
    if len(header) < 32:
        return None
    frame += header

    total_len = struct.unpack_from('<I', header, 4)[0]  # totalPacketLen
    remaining = total_len - (8 + 32)
    if remaining <= 0 or remaining > 200000:
        return None

    payload = ser.read(remaining)
    if len(payload) < remaining:
        return None
    frame += payload
    return bytes(frame)

# -------------------- Internal TLV parser (fallback) --------------------
TLV_TYPE_RANGE_PROFILE = 2
def parse_range_profile_from_packet(packet_bytes):
    """Return dict with at least {'rangeProfile': np.ndarray} if present."""
    if len(packet_bytes) < 40:
        return {}
    version, total_len, platform, frame_num, time_cpu, nObj, nTLV, subframe = struct.unpack_from('<8I', packet_bytes, 8)
    offset = 8 + 32
    out = {}
    for _ in range(int(nTLV)):
        if offset + 8 > len(packet_bytes):
            break
        tlv_type, tlv_len = struct.unpack_from('<2I', packet_bytes, offset)
        offset += 8
        payload = packet_bytes[offset : offset + tlv_len - 8]
        offset += (tlv_len - 8)
        if tlv_type == TLV_TYPE_RANGE_PROFILE and len(payload) >= 2:
            prof = np.frombuffer(payload, dtype=np.uint16).astype(np.float32)
            out['rangeProfile'] = prof
    return out

# -------------------- Breathing estimation --------------------
def bandpass_or_detrend(x, fs, lo=0.08, hi=0.7):
    x = np.asarray(x, dtype=np.float64)
    x = x - np.mean(x)
    if HAVE_SCIPY and fs > 0:
        lo_n = max(lo/(fs/2), 1e-4); hi_n = min(hi/(fs/2), 0.99)
        if lo_n < hi_n:
            b,a = butter(2, [lo_n, hi_n], btype='bandpass')
            return filtfilt(b, a, x)
    # fallback: 1 s moving-average detrend
    win = max(3, int(round(fs*1.0)))
    if win % 2 == 0: win += 1
    ma = np.convolve(x, np.ones(win)/win, mode='same')
    return x - ma

def estimate_bpm(sig, fs, band=(0.08, 0.7)):
    """Return (bpm, peak_hz); None if not enough data."""
    if fs <= 0 or len(sig) < 16:
        return None, None
    x = bandpass_or_detrend(sig, fs, band[0], band[1])
    n = int(2**np.ceil(np.log2(len(x))))
    X = np.fft.rfft(x, n=n)
    f = np.fft.rfftfreq(n, d=1.0/fs)
    mag = np.abs(X)
    m = (f >= band[0]) & (f <= band[1])
    if not np.any(m):
        return None, None
    pk = np.argmax(mag[m]); idx = np.where(m)[0][pk]
    peak_hz = f[idx]
    return 60.0 * peak_hz, peak_hz

# -------------------- Continuous monitor + plotting --------------------
def stream_breath_monitor(
    cli_port, data_port, cfg_path,
    known_fps=10.0, window_s=40.0, update_every_s=2.0,
    ignore_first_bins=6, ignore_last_bins=6, ewma_alpha=0.25,
    band_hz=(0.08, 0.7)
):
    # open serials
    cli = serial.Serial(cli_port, 115200, parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE, timeout=0.6)
    data = serial.Serial(data_port, 921600, parity=serial.PARITY_NONE,
                         stopbits=serial.STOPBITS_ONE, timeout=0.6)
    cli.reset_input_buffer(); cli.reset_output_buffer()
    data.reset_input_buffer(); data.reset_output_buffer()
    print(f"[UART] Connected: CLI={cli_port}@115200  DATA={data_port}@921600")

    # stop then send cfg
    cli.write(b"sensorStop\n")
    _ = read_cli_until_quiet(cli, 200)
    send_cfg(cli, cfg_path)

    # buffers (store ~2*window for safety)
    cap_len = max(100, int((known_fps or 10.0) * window_s * 2))
    amp = deque(maxlen=cap_len)  # dominant-bin amplitude
    ts  = deque(maxlen=cap_len)  # timestamps
    bpm_smooth = None
    last_est = time.time()

    # ---- live plot setup ----
    plt.ion()
    fig = plt.figure("IWR1843 Breathing Monitor", figsize=(8, 4.5))
    ax = fig.add_subplot(111)
    line, = ax.plot([], [], lw=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Dominant-bin amplitude")
    ax.set_title("Dominant range-bin amplitude (last window)")
    ax.grid(True)
    fig.canvas.flush_events()

    t0 = time.time()
    print(f"[RUN] Streaming: window={window_s:.0f}s, update every {update_every_s:.0f}s (Ctrl+C to stop)")

    try:
        while True:
            pkt = read_one_frame(data)
            now = time.time()
            if pkt is None:
                # no data this attempt; update GUI occasionally to keep responsive
                plt.pause(0.001)
                continue

            # parse range profile
            if USE_PARSEFRAME and parseStandardFrame is not None:
                out = parseStandardFrame(pkt) or {}
                prof = (out.get("rangeProfile") or out.get("RangeProfile"))
            else:
                out = parse_range_profile_from_packet(pkt)
                prof = out.get("rangeProfile")

            if prof is None or len(prof) < 8:
                plt.pause(0.001)
                continue
            prof = np.asarray(prof, dtype=np.float32)

            # dominant static bin
            start = max(0, ignore_first_bins)
            end = max(start+1, len(prof)-max(0, ignore_last_bins))
            roi = prof[start:end]
            idx = start + int(np.argmax(roi))

            amp.append(float(prof[idx]))
            ts.append(now)

            # estimate sampling rate
            if known_fps and known_fps > 0:
                fs = float(known_fps)
            else:
                if len(ts) >= 5:
                    dt = np.diff(np.array(ts))
                    dt = dt[dt > 0]
                    fs = 1.0/np.median(dt) if len(dt) else 10.0
                else:
                    fs = 10.0

            # periodic BPM update using last window
            if (now - last_est) >= update_every_s and len(amp) > max(16, int(fs*6)):
                last_est = now
                t_arr = np.array(ts)
                y_arr = np.array(amp)
                cutoff = now - window_s
                m = t_arr >= cutoff
                t_win = t_arr[m]; y_win = y_arr[m]
                if len(y_win) > max(16, int(fs*10)):
                    bpm, peak_hz = estimate_bpm(y_win, fs, band=band_hz)
                    if bpm is not None:
                        bpm_smooth = bpm if bpm_smooth is None else (ewma_alpha*bpm + (1.0-ewma_alpha)*bpm_smooth)
                        print(f"[{time.strftime('%H:%M:%S')}] frames={len(y_win):4d} "
                              f"fs≈{fs:4.1f}Hz  bin={idx:3d}  BPM={bpm:5.1f}  (smooth={bpm_smooth:5.1f})")

                # update plot to show last window
                t_rel = t_win - t0
                line.set_data(t_rel, y_win)
                if len(t_rel) >= 2:
                    ax.set_xlim(max(0, t_rel[-1]-window_s), t_rel[-1] + 0.5)
                    ymin = np.min(y_win); ymax = np.max(y_win)
                    if ymin == ymax:
                        ymin -= 1; ymax += 1
                    pad = 0.05*(ymax - ymin)
                    ax.set_ylim(ymin - pad, ymax + pad)
                fig.canvas.draw_idle()
                plt.pause(0.001)  # keep UI responsive

    except KeyboardInterrupt:
        print("\n[INFO] Keyboard interrupt — stopping sensor.")
    finally:
        try:
            cli.write(b"sensorStop\n")
            _ = read_cli_until_quiet(cli, 200)
        except Exception:
            pass
        try:
            data.close(); cli.close()
        except Exception:
            pass
        print("[UART] Closed.")
        try:
            plt.ioff()
            plt.show(block=False)
        except Exception:
            pass

# -------------------- CLI entry --------------------
def main():
    import argparse
    ap = argparse.ArgumentParser(description="IWR1843 breathing monitor (SDK OOB Demo) + Live Plot")
    ap.add_argument("--cli", default="COM7", help="CLI COM port (e.g., COM5 or /dev/ttyUSB0)")
    ap.add_argument("--data", default="COM8", help="DATA COM port (e.g., COM6 or /dev/ttyUSB1)")
    ap.add_argument("--cfg", default="xwr18xx_profile_2023_12_18T13_32_44_644.cfg", help="Path to .cfg file")
    ap.add_argument("--fps", type=float, default=10.0, help="Known FPS (set None to auto-estimate)")
    ap.add_argument("--window", type=float, default=10.0, help="Sliding window seconds for FFT/plot")
    ap.add_argument("--update", type=float, default=2.0, help="Update period seconds")
    ap.add_argument("--band_lo", type=float, default=0.08, help="Breath band low Hz")
    ap.add_argument("--band_hi", type=float, default=0.70, help="Breath band high Hz")
    ap.add_argument("--ignore_head", type=int, default=1, help="Ignore first N range bins")
    ap.add_argument("--ignore_tail", type=int, default=10, help="Ignore last N range bins")
    args = ap.parse_args()

    stream_breath_monitor(
        cli_port=args.cli,
        data_port=args.data,
        cfg_path=args.cfg,
        known_fps=args.fps,
        window_s=args.window,
        update_every_s=args.update,
        ignore_first_bins=args.ignore_head,
        ignore_last_bins=args.ignore_tail,
        band_hz=(args.band_lo, args.band_hi),
    )

if __name__ == "__main__":
    main()
