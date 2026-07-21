#!/usr/bin/env python3
"""Liest UBX-NAV-PVT Nachrichten vom ZED-F9P und druckt die fuer
das GNSS-Extended-Context-Datenmodell relevanten Felder."""

import serial
from pyubx2 import UBXReader

PORT = "/dev/ttyACM0"
BAUD = 38400  # u-blox Werksdefault fuer USB/UART, ggf. anpassen

with serial.Serial(PORT, BAUD, timeout=3) as stream:
    reader = UBXReader(stream)
    for (raw, msg) in reader:
        if msg is None:
            continue
        if msg.identity == "NAV-PVT":
            print(
                f"Fix: {msg.fixType}  Sats: {msg.numSV}  "
                f"Lat: {msg.lat:.7f}  Lon: {msg.lon:.7f}  "
                f"Height: {msg.height/1000:.2f} m  "
                f"hAcc: {msg.hAcc/1000:.2f} m  vAcc: {msg.vAcc/1000:.2f} m  "
                f"Zeit: {msg.year:04d}-{msg.month:02d}-{msg.day:02d} "
                f"{msg.hour:02d}:{msg.min:02d}:{msg.second:02d}"
            )
