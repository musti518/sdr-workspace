// gnss_to_vrt: liest UBX-NAV-PVT vom u-blox ZED-F9P (seriell) und sendet
// die Werte als VRT Extended-Context-Pakete (OUI 0xFF0044) ueber ZeroMQ.
// Gedacht zum Einbinden in eine bestehende Quelle via --merge-port.

#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <zmq.h>

#include <vrt/vrt_types.h>
#include <vrt/vrt_write.h>
#include <vrt/vrt_string.h>
#include <vrt/vrt_init.h>

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int) { stop_signal_called = true; }

// ---------------------------------------------------------------------
// Serielle Schnittstelle oeffnen (8N1, keine Flusskontrolle)
// ---------------------------------------------------------------------
int open_serial(const std::string& device, int baud) {
    int fd = open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Konnte " << device << " nicht oeffnen: "
                  << strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr fehlgeschlagen: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    speed_t speed;
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:     speed = B38400;  break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag = 0;

    tty.c_cc[VMIN]  = 1;   // blockierend, mind. 1 Byte
    tty.c_cc[VTIME] = 5;   // 0.5s Timeout zwischen Bytes

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr fehlgeschlagen: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }
    return fd;
}

// liest genau n Bytes, blockierend, mit einfachem Timeout-Schutz
static bool read_exact(int fd, uint8_t* buf, size_t n) {
    size_t got = 0;
    while (got < n) {
        ssize_t r = read(fd, buf + got, n - got);
        if (r <= 0) {
            if (stop_signal_called) return false;
            continue; // Timeout einzelner read()-Aufrufe: einfach weiterversuchen
        }
        got += r;
    }
    return true;
}

// ---------------------------------------------------------------------
// UBX-NAV-PVT Struktur (Auszug, u-blox Interface Description, stabil)
// Nachricht: Klasse 0x01, ID 0x07, Laenge 92 Byte
// ---------------------------------------------------------------------
#pragma pack(push, 1)
struct ubx_nav_pvt_payload {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;      // 1e-7 Grad
    int32_t  lat;      // 1e-7 Grad
    int32_t  height;   // mm, ueber Ellipsoid
    int32_t  hMSL;     // mm, ueber Meeresspiegel
    uint32_t hAcc;     // mm
    uint32_t vAcc;     // mm
    int32_t  velN, velE, velD; // mm/s
    int32_t  gSpeed;   // mm/s
    int32_t  headMot;  // 1e-5 Grad
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;     // 0.01
    // Rest (reserved, headVeh, magDec, magAcc) hier nicht benoetigt
};
#pragma pack(pop)

// UBX-Checksumme (8-Bit Fletcher, wie im Interface-Manual spezifiziert)
static void ubx_checksum(const uint8_t* data, size_t len, uint8_t& ck_a, uint8_t& ck_b) {
    ck_a = 0; ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        ck_a = ck_a + data[i];
        ck_b = ck_b + ck_a;
    }
}

// Sucht im seriellen Strom das naechste vollstaendige, gueltige
// UBX-NAV-PVT-Paket und befuellt payload. Blockiert, bis eins da ist.
static bool read_next_nav_pvt(int fd, ubx_nav_pvt_payload* payload) {
    uint8_t b;
    for (;;) {
        if (stop_signal_called) return false;

        // Sync-Byte 1: 0xB5
        if (!read_exact(fd, &b, 1)) return false;
        if (b != 0xB5) continue;

        // Sync-Byte 2: 0x62
        if (!read_exact(fd, &b, 1)) return false;
        if (b != 0x62) continue;

        uint8_t header[4]; // class, id, len_lo, len_hi
        if (!read_exact(fd, header, 4)) return false;
        uint8_t msg_class = header[0];
        uint8_t msg_id    = header[1];
        uint16_t len      = header[2] | (header[3] << 8);

        if (len > 512) continue; // Schutz gegen Fehlsynchronisation

        std::vector<uint8_t> body(len);
        if (len > 0 and !read_exact(fd, body.data(), len)) return false;

        uint8_t ck_a_rx, ck_b_rx;
        if (!read_exact(fd, &ck_a_rx, 1)) return false;
        if (!read_exact(fd, &ck_b_rx, 1)) return false;

        std::vector<uint8_t> ck_input;
        ck_input.push_back(header[0]);
        ck_input.push_back(header[1]);
        ck_input.push_back(header[2]);
        ck_input.push_back(header[3]);
        ck_input.insert(ck_input.end(), body.begin(), body.end());
        uint8_t ck_a, ck_b;
        ubx_checksum(ck_input.data(), ck_input.size(), ck_a, ck_b);
        if (ck_a != ck_a_rx or ck_b != ck_b_rx) continue; // Checksumme falsch, verwerfen

        if (msg_class == 0x01 and msg_id == 0x07 and len >= sizeof(ubx_nav_pvt_payload)) {
            memcpy(payload, body.data(), sizeof(ubx_nav_pvt_payload));
            return true;
        }
        // Andere Nachricht (z.B. NMEA-Reste, andere UBX-Klassen): ignorieren, weitersuchen
    }
}

// ---------------------------------------------------------------------
// UBX-Ausgabe von NAV-PVT aktivieren (falls das Modul werksseitig nur
// NMEA sendet). CFG-MSG: Klasse 0x06, ID 0x01.
// ---------------------------------------------------------------------
static void enable_nav_pvt(int fd) {
    // CFG-MSG mit expliziten Raten fuer alle 6 Ports:
    // I2C, UART1, UART2, USB, SPI, reserved -> je 1 = jede Epoche senden
    uint8_t msg[] = {
        0xB5, 0x62,             // Sync
        0x06, 0x01,             // CFG-MSG
        0x08, 0x00,             // Laenge 8
        0x01, 0x07,             // Klasse/ID: NAV-PVT
        0x01, 0x01, 0x01, 0x01, 0x01, 0x01, // Raten: I2C,UART1,UART2,USB,SPI,res
        0x00, 0x00              // Checksum-Platzhalter
    };
    uint8_t ck_a, ck_b;
    ubx_checksum(msg + 2, 12, ck_a, ck_b); // ab class bis Ende Payload (2+2+8=12 Byte)
    msg[14] = ck_a;
    msg[15] = ck_b;
    write(fd, msg, sizeof(msg));
    tcdrain(fd); // sicherstellen, dass es tatsaechlich rausgeht, bevor wir weitermachen
    usleep(100000); // dem Modul kurz Zeit geben, die Konfiguration zu uebernehmen
}
    uint8_t ck_a, ck_b;

int main(int argc, char* argv[]) {

    std::string device, address;
    int baud, port;
    double update_interval;
    bool progress;

    po::options_description desc("gnss_to_vrt: UBX-NAV-PVT to VRT Extended Context. Allowed options");
    desc.add_options()
        ("help", "help message")
        ("device", po::value<std::string>(&device)->default_value("/dev/ttyACM0"),
         "serial device of the GNSS receiver")
        ("baud", po::value<int>(&baud)->default_value(38400), "serial baud rate")
        ("port", po::value<int>(&port)->default_value(50110), "VRT ZMQ port")
        ("address", po::value<std::string>(&address)->default_value("*"), "VRT ZMQ bind address")
        ("progress", po::bool_switch(&progress)->default_value(false),
         "periodically display received fixes");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return ~0;
    }

    std::signal(SIGINT, sig_int_handler);
    std::signal(SIGTERM, sig_int_handler);

    int fd = open_serial(device, baud);
    if (fd < 0) return 1;

    enable_nav_pvt(fd);

    void* context = zmq_ctx_new();
    void* publisher = zmq_socket(context, ZMQ_PUB);
    std::string endpoint = str(boost::format("tcp://%s:%d") % address % port);
    int rc = zmq_bind(publisher, endpoint.c_str());
    if (rc != 0) {
        std::cerr << "zmq_bind fehlgeschlagen auf " << endpoint << std::endl;
        return 1;
    }
    std::cout << "gnss_to_vrt: sende auf " << endpoint
              << " (Geraet " << device << ", " << baud << " Baud)" << std::endl;
    std::cout << "Press Ctrl + C to stop..." << std::endl;

    // kurze Anlaufzeit fuer den PUB-Socket (ZeroMQ "slow joiner")
    usleep(200000);

    uint32_t buffer[64]; // Header+Fields+20 Worte Body reicht bequem

    while (not stop_signal_called) {

        ubx_nav_pvt_payload pvt;
        if (!read_next_nav_pvt(fd, &pvt)) {
            if (stop_signal_called) break;
            continue;
        }

        // aktuelle Systemzeit fuer den VRT-Paket-Zeitstempel (nicht die GNSS-Zeit!)
        auto now = std::chrono::system_clock::now();
        uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                               now.time_since_epoch()).count();
        uint32_t ts_int = now_us / 1000000;
        uint64_t ts_frac_ps = (uint64_t)(now_us % 1000000) * 1000000; // us -> ps

        // ---- 20-Worte-Nutzlast nach dem Layout aus gnss-extended-context.h ----
        uint32_t payload[20];
        memset(payload, 0, sizeof(payload));

        uint32_t fix_type = pvt.fixType;
        memcpy(&payload[0], &fix_type, sizeof(uint32_t));
        uint32_t num_sv = pvt.numSV;
        memcpy(&payload[1], &num_sv, sizeof(uint32_t));

        double latitude  = pvt.lat / 1e7;
        double longitude = pvt.lon / 1e7;
        memcpy(&payload[2], &latitude, sizeof(double));
        memcpy(&payload[4], &longitude, sizeof(double));

        float height_m = pvt.height / 1000.0f;
        float h_acc_m  = pvt.hAcc / 1000.0f;
        float v_acc_m  = pvt.vAcc / 1000.0f;
        memcpy(&payload[6], &height_m, sizeof(float));
        memcpy(&payload[7], &h_acc_m, sizeof(float));
        memcpy(&payload[8], &v_acc_m, sizeof(float));

        float ground_speed = pvt.gSpeed / 1000.0f;
        float heading = pvt.headMot / 1e5f;
        float pdop = pvt.pDOP / 100.0f;
        memcpy(&payload[9], &ground_speed, sizeof(float));
        memcpy(&payload[10], &heading, sizeof(float));
        memcpy(&payload[11], &pdop, sizeof(float));

        uint32_t flags = 0;
        if (pvt.valid & 0x01) flags |= (1<<0); // validDate
        if (pvt.valid & 0x02) flags |= (1<<1); // validTime
        if (pvt.valid & 0x04) flags |= (1<<2); // fullyResolved
        if (pvt.flags & 0x01) flags |= (1<<3); // gnssFixOK
        if (pvt.flags & 0x02) flags |= (1<<4); // diffSoln
        memcpy(&payload[12], &flags, sizeof(uint32_t));

        uint32_t year = pvt.year, month = pvt.month, day = pvt.day;
        uint32_t hour = pvt.hour, minute = pvt.min, sec = pvt.sec;
        memcpy(&payload[13], &year, sizeof(uint32_t));
        memcpy(&payload[14], &month, sizeof(uint32_t));
        memcpy(&payload[15], &day, sizeof(uint32_t));
        memcpy(&payload[16], &hour, sizeof(uint32_t));
        memcpy(&payload[17], &minute, sizeof(uint32_t));
        memcpy(&payload[18], &sec, sizeof(uint32_t));
        memcpy(&payload[19], &pvt.nano, sizeof(int32_t));

        // ---- VRT-Paket zusammenbauen ----
        struct vrt_packet p;
        vrt_init_packet(&p);

        p.header.packet_type = VRT_PT_EXT_CONTEXT;
        p.header.tsi = VRT_TSI_UTC;
        p.header.tsf = VRT_TSF_REAL_TIME;
        p.header.has.class_id = true;

        p.fields.stream_id = 1;
        p.fields.class_id.oui = 0xFF0044; // eigene OUI, analog dt (0x42) / tracker (0x43)
        p.fields.class_id.information_class_code = 0;
        p.fields.class_id.packet_class_code = 0;
        p.fields.integer_seconds_timestamp = ts_int;
        p.fields.fractional_seconds_timestamp = ts_frac_ps;

        p.body = (void*)payload;
        p.words_body = 20;

        int32_t rv = vrt_write_packet(&p, buffer, 64, true);
        if (rv < 0) {
            std::cerr << "Failed to write packet: " << vrt_string_error(rv) << std::endl;
            continue;
        }

        zmq_send(publisher, buffer, rv * 4, 0);

        if (progress) {
            std::cout << boost::format(
                "Fix: %u  Sats: %u  Lat: %.7f  Lon: %.7f  hAcc: %.2fm  vAcc: %.2fm  "
                "Zeit: %04u-%02u-%02u %02u:%02u:%02u")
                % fix_type % num_sv % latitude % longitude % h_acc_m % v_acc_m
                % year % month % day % hour % minute % sec
                << std::endl;
        }
    }

    close(fd);
    zmq_close(publisher);
    zmq_ctx_destroy(context);
    return 0;
}
