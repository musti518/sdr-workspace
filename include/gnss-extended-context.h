#ifndef _GNSSEXTENDEDCONTEXT_H
#define _GNSSEXTENDEDCONTEXT_H

struct gnss_ext_context_type {
    bool gnss_ext_context_received = false;

    uint32_t fix_type = 0;      // 0=kein Fix, 2=2D, 3=3D, 4=GNSS+Koppelnavigation
    uint32_t num_sv = 0;        // Satelliten in der Loesung

    double latitude = NAN;      // Grad, WGS84
    double longitude = NAN;     // Grad, WGS84
    float height = NAN;         // Meter ueber WGS84-Ellipsoid
    float h_acc = NAN;          // horizontale Genauigkeit, Meter (1 sigma)
    float v_acc = NAN;          // vertikale Genauigkeit, Meter (1 sigma)

    float ground_speed = NAN;   // m/s
    float heading = NAN;        // Grad
    float pdop = NAN;           // Position Dilution of Precision

    bool valid_date = false;
    bool valid_time = false;
    bool fully_resolved = false;
    bool gnss_fix_ok = false;
    bool diff_soln = false;     // DGPS/RTK-Korrektur aktiv

    uint32_t gnss_year = 0;
    uint32_t gnss_month = 0;
    uint32_t gnss_day = 0;
    uint32_t gnss_hour = 0;
    uint32_t gnss_min = 0;
    uint32_t gnss_sec = 0;
    int32_t  gnss_nanosecond = 0; // Sub-Sekunden-Anteil, kann negativ sein (ublox-Eigenheit)

    uint32_t stream_id;
    uint64_t fractional_seconds_timestamp;
    uint64_t integer_seconds_timestamp;
};

// Wort-Layout ab vrt_packet->offset (1 Wort = 4 Byte):
//  0    fix_type       (uint32_t)
//  1    num_sv         (uint32_t)
//  2-3  latitude       (double)
//  4-5  longitude      (double)
//  6    height         (float)
//  7    h_acc          (float)
//  8    v_acc          (float)
//  9    ground_speed   (float)
//  10   heading        (float)
//  11   pdop           (float)
//  12   flags          (uint32_t, Bitfeld)
//  13   gnss_year      (uint32_t)
//  14   gnss_month     (uint32_t)
//  15   gnss_day       (uint32_t)
//  16   gnss_hour      (uint32_t)
//  17   gnss_min       (uint32_t)
//  18   gnss_sec       (uint32_t)
//  19   gnss_nanosecond (int32_t)
//  -> 20 Worte Nutzlast insgesamt

bool gnss_process(uint32_t* buffer, uint32_t size, packet_type* vrt_packet, gnss_ext_context_type* gnss_ext_context) {

    if (vrt_packet->oui == 0xFF0044) { // eigene OUI, analog zu dt (0x42) / tracker (0x43)

        gnss_ext_context->stream_id = vrt_packet->stream_id;
        gnss_ext_context->fractional_seconds_timestamp = vrt_packet->fractional_seconds_timestamp;
        gnss_ext_context->integer_seconds_timestamp = vrt_packet->integer_seconds_timestamp;

        memcpy(&gnss_ext_context->fix_type,     (char*)&buffer[vrt_packet->offset],    sizeof(uint32_t));
        memcpy(&gnss_ext_context->num_sv,       (char*)&buffer[vrt_packet->offset+1],  sizeof(uint32_t));
        memcpy(&gnss_ext_context->latitude,     (char*)&buffer[vrt_packet->offset+2],  sizeof(double));
        memcpy(&gnss_ext_context->longitude,    (char*)&buffer[vrt_packet->offset+4],  sizeof(double));
        memcpy(&gnss_ext_context->height,       (char*)&buffer[vrt_packet->offset+6],  sizeof(float));
        memcpy(&gnss_ext_context->h_acc,        (char*)&buffer[vrt_packet->offset+7],  sizeof(float));
        memcpy(&gnss_ext_context->v_acc,        (char*)&buffer[vrt_packet->offset+8],  sizeof(float));
        memcpy(&gnss_ext_context->ground_speed, (char*)&buffer[vrt_packet->offset+9],  sizeof(float));
        memcpy(&gnss_ext_context->heading,      (char*)&buffer[vrt_packet->offset+10], sizeof(float));
        memcpy(&gnss_ext_context->pdop,         (char*)&buffer[vrt_packet->offset+11], sizeof(float));

        uint32_t flags = buffer[vrt_packet->offset+12];
        gnss_ext_context->valid_date     = flags & (1<<0);
        gnss_ext_context->valid_time     = flags & (1<<1);
        gnss_ext_context->fully_resolved = flags & (1<<2);
        gnss_ext_context->gnss_fix_ok    = flags & (1<<3);
        gnss_ext_context->diff_soln      = flags & (1<<4);

        memcpy(&gnss_ext_context->gnss_year,       (char*)&buffer[vrt_packet->offset+13], sizeof(uint32_t));
        memcpy(&gnss_ext_context->gnss_month,      (char*)&buffer[vrt_packet->offset+14], sizeof(uint32_t));
        memcpy(&gnss_ext_context->gnss_day,        (char*)&buffer[vrt_packet->offset+15], sizeof(uint32_t));
        memcpy(&gnss_ext_context->gnss_hour,       (char*)&buffer[vrt_packet->offset+16], sizeof(uint32_t));
        memcpy(&gnss_ext_context->gnss_min,        (char*)&buffer[vrt_packet->offset+17], sizeof(uint32_t));
        memcpy(&gnss_ext_context->gnss_sec,        (char*)&buffer[vrt_packet->offset+18], sizeof(uint32_t));
        memcpy(&gnss_ext_context->gnss_nanosecond, (char*)&buffer[vrt_packet->offset+19], sizeof(int32_t));

        gnss_ext_context->gnss_ext_context_received = true;
        return true;
    } else
        return false;
}

#endif
