#include "mobility_packets.h"
#include <string.h>

#ifndef __ORDER_LITTLE_ENDIAN__
#include <endian.h>  /* for htole32, le32toh etc. */
#else 

static inline uint8_t htole8(uint8_t a) {
    return a;
}

static inline uint16_t htole16(uint16_t a) {
    return a;
}

static inline uint32_t htole32(uint32_t a) {
    return a;
}

static inline uint64_t htole64(uint64_t a) {
    return a;
}

static inline uint8_t le8toh(uint8_t a) {
    return a;
}

static inline uint16_t le16toh(uint16_t a) {
    return a;
}

static inline uint32_t le32toh(uint32_t a) {
    return a;
}

static inline uint64_t le64toh(uint64_t a) {
    return a;
}

#endif

/* ==============================
 * CRC32 Implementation (Polynomial 0xEDB88320)
 * ============================== */
uint32_t mobility_crc32(const void* data, size_t len) {
    static uint32_t table[256];
    static bool init = false;
    if (!init) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (size_t j = 0; j < 8; j++)
                c = (c & 1) ? (0xEDB88320U ^ (c >> 1)) : (c >> 1);
            table[i] = c;
        }
        init = true;
    }
    uint32_t crc = 0xFFFFFFFFU;
    const uint8_t* p = (const uint8_t*) data;
    for (size_t i = 0; i < len; i++)
        crc = table[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFU;
}

/* ==============================
 * Sequence Generator
 * ============================== */
uint16_t mobility_next_sequence(void) {
    static uint16_t seq = 0;
    return ++seq;
}

/* ==============================
 * Helpers
 * ============================== */
size_t mobility_packet_header_size(void) {
    return sizeof(MobilityPacketHeader);
}

size_t mobility_packet_motor_floats_size(void) {
    return mobility_packet_header_size() + sizeof(float) * MOBILITY_MOTOR_COUNT;
}

size_t mobility_packet_enable_size(void) {
    return mobility_packet_header_size() + sizeof(uint8_t);
}

size_t mobility_packet_size_for_motor_velocities(void) {
    return mobility_packet_motor_floats_size();
}

size_t mobility_packet_size_for_motor_duties(void) {
    return mobility_packet_motor_floats_size();
}


size_t mobility_packet_size_for_request(void) {
    return mobility_packet_header_size() + sizeof(uint32_t);
}

size_t mobility_packet_size_for_init(void) {
    return mobility_packet_header_size() +
        sizeof(MobilityInitPacket);
}

size_t mobility_packet_size_for_device_state(void) {
    return mobility_packet_header_size() +
        sizeof(MobilityDeviceState);
}

size_t mobility_packet_size_for_feedback(void) {
    return mobility_packet_header_size() +
        sizeof(MobilityFeedback);
}


/* ==============================
 * Build Functions
 * ============================== */
bool mobility_pack_motor_velocities_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us,
    const float velocities[MOBILITY_MOTOR_COUNT]) {
    if (!buf || !velocities) return false;
    size_t payload_size = sizeof(float) * MOBILITY_MOTOR_COUNT;
    size_t total_size = mobility_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    MobilityPacketHeader hdr;
    hdr.device_id = htole32(MOBILITY_DEVICE_ID);
    hdr.command_id = htole16(set_velocity);
    hdr.sequence = htole16(mobility_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), velocities, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        mobility_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool mobility_pack_motor_dutycycles_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us,
    const float duties[MOBILITY_MOTOR_COUNT]) {
    if (!buf || !duties) return false;
    size_t payload_size = sizeof(float) * MOBILITY_MOTOR_COUNT;
    size_t total_size = mobility_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    MobilityPacketHeader hdr;
    hdr.device_id = htole32(MOBILITY_DEVICE_ID);
    hdr.command_id = htole16(set_dutycycle);
    hdr.sequence = htole16(mobility_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), duties, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        mobility_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool mobility_pack_request_device_state_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us) {
    if (!buf) return false;
    size_t payload_size = sizeof(uint32_t);
    size_t total_size = mobility_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    MobilityPacketHeader hdr;
    hdr.device_id = htole32(MOBILITY_DEVICE_ID);
    hdr.command_id = htole16(request_device_state);
    hdr.sequence = htole16(mobility_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    uint32_t payload_data = 0x69;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &payload_data, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        mobility_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool mobility_pack_request_feedback_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us) {
    if (!buf) return false;
    size_t payload_size = sizeof(uint32_t);
    size_t total_size = mobility_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    MobilityPacketHeader hdr;
    hdr.device_id = htole32(MOBILITY_DEVICE_ID);
    hdr.command_id = htole16(request_motor_feedback);
    hdr.sequence = htole16(mobility_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    uint32_t payload_data = 0x69;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), &payload_data, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        mobility_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool mobility_pack_init_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us,
    const MobilityInitPacket* init) {
    if (!buf || !init) return false;
    size_t payload_size = sizeof(MobilityInitPacket);
    size_t total_size = mobility_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    MobilityPacketHeader hdr;
    hdr.device_id = htole32(MOBILITY_DEVICE_ID);
    hdr.command_id = htole16(init_device);
    hdr.sequence = htole16(mobility_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), init, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        mobility_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool mobility_pack_enable_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us, MobilityCommands command, bool enable) {
    if (!buf) return false;
    size_t payload_size = sizeof(uint8_t);
    size_t total_size = mobility_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    MobilityPacketHeader hdr;
    hdr.device_id = htole32(MOBILITY_DEVICE_ID);
    hdr.command_id = htole16(command);
    hdr.sequence = htole16(mobility_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    uint8_t data = (enable);
    memcpy(buf + sizeof(hdr), &data, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        mobility_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool mobility_pack_device_state_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us,
    const MobilityDeviceState* state) {
    if (!buf || !state) return false;
    size_t payload_size = sizeof(MobilityDeviceState);
    size_t total_size = mobility_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    MobilityPacketHeader hdr;
    hdr.device_id = htole32(MOBILITY_DEVICE_ID);
    hdr.command_id = htole16(device_state);
    hdr.sequence = htole16(mobility_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), state, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        mobility_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

bool mobility_pack_feedback_packet(uint8_t* buf, size_t buf_size,
    uint64_t time_since_boot_us,
    const MobilityFeedback* feedback) {
    if (!buf || !feedback) return false;
    size_t payload_size = sizeof(MobilityFeedback);
    size_t total_size = mobility_packet_header_size() + payload_size;
    if (buf_size < total_size) return false;

    MobilityPacketHeader hdr;
    hdr.device_id = htole32(MOBILITY_DEVICE_ID);
    hdr.command_id = htole16(motor_feedback);
    hdr.sequence = htole16(mobility_next_sequence());
    hdr.content_size = htole16(payload_size);
    hdr.time_since_boot_us = htole64(time_since_boot_us);
    hdr.crc32 = 0;

    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), feedback, payload_size);

    /* compute CRC of header (without CRC field) + payload */
    hdr.crc32 = htole32(
        mobility_crc32(buf, total_size - sizeof(hdr.crc32))
    );

    memcpy(buf, &hdr, sizeof(hdr));
    return true;
}

/* ==============================
 * Unpacking
 * ============================== */
bool mobility_unpack_packet_header(const uint8_t* buf, size_t buf_size, MobilityPacketHeader* out_header) {
    if (!buf || buf_size < mobility_packet_header_size())
        return false;

    MobilityPacketHeader hdr;
    memcpy(&hdr, buf, sizeof(hdr));

    uint32_t crc_recv = le32toh(hdr.crc32);
    uint16_t content_size = le16toh(hdr.content_size);

    size_t expected_total_size = mobility_packet_header_size() + content_size;

    // Zero out the CRC field for verification
    MobilityPacketHeader temp_hdr = hdr;
    temp_hdr.crc32 = 0;

    uint8_t temp_buf[sizeof(MobilityPacketHeader)] = { 0 };
    memcpy((void*) temp_buf, &temp_hdr, sizeof(temp_hdr));

    if (buf_size > expected_total_size) {

        uint32_t crc_calc = mobility_crc32(temp_buf, expected_total_size - sizeof(hdr.crc32));
        if (crc_calc != crc_recv)
            return false; // CRC mismatch
    }

    if (out_header) {
        hdr.device_id = le32toh(hdr.device_id);
        hdr.command_id = le16toh(hdr.command_id);
        hdr.sequence = le16toh(hdr.sequence);
        hdr.content_size = content_size;
        hdr.time_since_boot_us = le64toh(hdr.time_since_boot_us);
        hdr.crc32 = crc_recv;
        *out_header = hdr;
    }

    return true;
}

bool mobility_unpack_motor_velocities_from_payload(const uint8_t* payload, size_t payload_size,
    float out_velocities[MOBILITY_MOTOR_COUNT]) {
    if (!payload || !out_velocities) return false;
    if (payload_size != sizeof(float) * MOBILITY_MOTOR_COUNT) return false;

    memcpy(out_velocities, payload,
        sizeof(float) * MOBILITY_MOTOR_COUNT);

    return true;
}

bool mobility_unpack_motor_dutycycles_from_payload(const uint8_t* payload, size_t payload_size,
    float out_duties[MOBILITY_MOTOR_COUNT]) {
    if (!payload || !out_duties) return false;
    if (payload_size != sizeof(float) * MOBILITY_MOTOR_COUNT) return false;

    memcpy(out_duties, payload,
        sizeof(float) * MOBILITY_MOTOR_COUNT);

    return true;
}

bool mobility_unpack_init_packet_from_payload(const uint8_t* payload, size_t payload_size,
    MobilityInitPacket* out_init) {
    if (!payload || !out_init) return false;
    if (payload_size != sizeof(MobilityInitPacket)) return false;

    memcpy(out_init, payload,
        sizeof(MobilityInitPacket));

    return true;
}

bool mobility_unpack_enable_from_payload(const uint8_t* payload, size_t payload_size,
    bool* out_enable) {
    if (!payload || !out_enable) return false;
    if (payload_size != sizeof(uint8_t)) return false;

    uint8_t data = 0;
    memcpy(&data, payload,
        sizeof(uint8_t));

    *out_enable = (bool) data;
    return true;
}

bool mobility_unpack_device_state_packet_from_payload(const uint8_t* payload, size_t payload_size,
    MobilityDeviceState* out_state) {
    if (!payload || !out_state) return false;
    if (payload_size != sizeof(MobilityDeviceState)) return false;

    memcpy(out_state, payload,
        sizeof(MobilityDeviceState));

    return true;
}

bool mobility_unpack_feedback_packet_from_payload(const uint8_t* payload, size_t payload_size,
    MobilityFeedback* out_feedback) {
    if (!payload || !out_feedback) return false;
    if (payload_size != sizeof(MobilityFeedback)) return false;

    memcpy(out_feedback, payload,
        sizeof(MobilityFeedback));

    return true;
}
