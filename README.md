
# sdr-workspace

Projekt für SDR-Arbeit mit VRT und SigMF. Basiert auf Thomas Telkamps
vrt-iq-tools und libvrt, erweitert um eine GNSS-Integration für präzise
Zeit- und Positionsangaben bei jeder Aufnahme.

## Worum geht's

Damit eine IQ-Aufnahme später ausgewertet werden kann, braucht sie einen
möglichst genauen Zeitstempel und – falls relevant – die Position der
Empfangsstation. Genau darum geht's hier: Funksignale aufnehmen und dabei
so viele exakte Zusatzinformationen wie möglich mitspeichern.

Vier Programme, die zusammenspielen:

- **`usrp_to_vrt`** – holt IQ-Daten vom USRP und schickt sie als VRT-Pakete
  übers Netzwerk (ZeroMQ)
- **`gnss_to_vrt`** – liest Position/Uhrzeit vom u-blox-GNSS-Modul aus und
  schickt sie im gleichen Format
- **`iq_recorder`** – hört beiden zu und schreibt am Ende zwei Dateien:
  die rohen Samples (`.sigmf-data`) und alle Metadaten als JSON
  (`.sigmf-meta`)
- **`vrt_to_rtl_tcp`** – falls man den Stream auch mal in GQRX o.ä.
  anschauen will

Die vier laufen als getrennte Prozesse und reden nur über ZeroMQ
miteinander, lokal auf dem gleichen Rechner. Gemeinsamer Code (VRT-Format,
GNSS-Verarbeitung, Tracking/Pointing – Letzteres ein Überbleibsel aus dem
ursprünglichen Teleskop-Projekt, von dem das hier abstammt) liegt in
`include/`.

## Struktur

- `src/*.cpp` – die vier Programme
- `include/*.h` – VRT-Basis + Extended-Context-Header (GNSS, Tracking, DT)
- `libvrt/` – Submodule für den VITA-49-Codec
- `gnss/read_pvt.py` – schnelles Python-Skript, um die GNSS-Hardware ohne
  den ganzen C++-Kram zu testen

## Hardware
<<<<<<< HEAD
=======
- Ettus USRP B205mini
- u-blox ZED-F9P, USB-Dongle, aktive Antenne
>>>>>>> 32bcf6cf821ec61aff90ae43c1a80e98d92063b9

- Ettus USRP B205mini
- u-blox ZED-F9P (GNSS), als USB-Dongle

## Bauen

```bash
git clone --recursive https://github.com/musti518/sdr-workspace.git
cd sdr-workspace && mkdir build && cd build
cmake .. && make -j$(nproc)
```

Wenn UHD nicht installiert ist, wird `usrp_to_vrt` einfach übersprungen
(nur eine Warnung, kein Fehler) – der Rest baut trotzdem durch.

## Benutzung

Reihenfolge: erst die Quelle(n) starten, dann den Recorder.

**Nur IQ, ohne GNSS:**

```bash
./usrp_to_vrt --freq <FREQ> --rate <RATE> --gain <GAIN> &
./iq_recorder --file <DATEINAME>
```

Läuft bis `Strg+C`, oder bis `--nsamps`/`--duration` erreicht ist.

**Mit GNSS:**

```bash
./gnss_to_vrt &
sleep 2
./iq_recorder --file <DATEINAME> --gnss &
sleep 1
./usrp_to_vrt --freq <FREQ> --rate <RATE> --gain <GAIN> \
    --merge 1 --merge-port 50110
```

Die `sleep`s sind nur da, damit jedes Programm Zeit hat, seine Verbindung
aufzubauen, bevor das nächste loslegt – ohne die kann's sonst Probleme
geben. `--merge` sorgt dafür, dass `usrp_to_vrt` die GNSS-Pakete mit
einsammelt und weiterreicht. Nicht vergessen: `--gnss` bei `iq_recorder`
dazuschreiben, sonst kommen die GNSS-Daten zwar an, landen aber nicht in
der `.meta`-Datei.

Das Ganze automatisch: `./start_gnss_test.sh <DATEINAME>` macht genau
das oben, nur ohne dass man's von Hand tippen muss.

**Ein paar Optionen, die immer nützlich sind:**

- `--progress` – zeigt live Durchsatz/Pegel
- `--continue` – bricht nicht gleich ab, wenn mal ein Paket verloren geht
- `--duration <SEK>` / `--nsamps <N>` – feste Länge statt manuell abbrechen
- `--author "..."` / `--description "..."` – landet mit im Meta-File

**Kontrollieren, ob's geklappt hat:**

```bash
cat <DATEINAME>.sigmf-meta
```

zeigt alles, was mitgeschrieben wurde – Frequenz, Sample-Rate, und falls
GNSS dabei war auch Position/Uhrzeit.

## TODO

- Geräte-Seriennummer noch nicht in die VRT-Kontextpakete eingebaut
```
