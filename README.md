# sdr-workspace

Arbeitsumgebung für SDR/VRT/SigMF-Entwicklung, basierend auf
Thomas Telkamps vrt-iq-tools und libvrt. Dient allen Tests und
Erweiterungen rund um IQ-Aufnahme, VRT-Metadaten und GNSS-Integration.

## Struktur
- src/iq_recorder.cpp              — VRT-Senke (Kopie von vrt_to_sigmf)
- include/*.h                      — vrt-tools + Extended-Context-Header (Kopien)
- include/gnss-extended-context.h  — eigene GNSS-Klasse (geplant)
- libvrt/                          — Submodule, VITA-49-Codec
- gnss/read_pvt.py                 — Referenz: Felder aus UBX-NAV-PVT (u-blox ZED-F9P)

## Hardware
- Ettus USRP B205mini (geplant)
- u-blox ZED-F9P, USB-Dongle, aktive Antenne

## Build
git clone --recursive <repo-url>
cd sdr-workspace && mkdir build && cd build && cmake .. && make -j$(nproc)
