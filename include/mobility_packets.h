#ifndef __MOBILITY_PACKETS_H__
#define __MOBILITY_PACKETS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "mobility_packet_types.h"


    /* ==============================
     * API
     * ============================== */

     /* --- Serialization helpers --- */
    uint16_t mobility_next_sequence(void);
    uint32_t mobility_crc32(const void* data, size_t len);

    /* --- Building packets --- */
    size_t mobility_packet_header_size(void);

    size_t mobility_packet_enable_size(void);
    size_t mobility_packet_motor_floats_size(void);

    size_t mobility_packet_size_for_request(void);
    size_t mobility_packet_size_for_motor_velocities(void);
    size_t mobility_packet_size_for_motor_duties(void);
    size_t mobility_packet_size_for_init(void);
    size_t mobility_packet_size_for_device_state(void);
    size_t mobility_packet_size_for_feedback(void);


    /* --- Packing --- */
    bool mobility_pack_motor_velocities_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us,
        const float velocities[MOBILITY_MOTOR_COUNT]);

    bool mobility_pack_motor_dutycycles_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us,
        const float duties[MOBILITY_MOTOR_COUNT]);

    bool mobility_pack_init_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us,
        const MobilityInitPacket* init);

    bool mobility_pack_enable_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us, MobilityCommands command, bool enable);

    bool mobility_pack_request_device_state_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us);

    bool mobility_pack_request_feedback_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us);

    bool mobility_pack_device_state_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us,
        const MobilityDeviceState* state);

    bool mobility_pack_feedback_packet(uint8_t* buf, size_t buf_size,
        uint64_t time_since_boot_us,
        const MobilityFeedback* feedback);

    /* --- Unpacking --- */

    bool mobility_unpack_packet_header(const uint8_t* buf, size_t buf_size,
        MobilityPacketHeader* out_header);

    /* --- Unpacking Payloads --- */

    bool mobility_unpack_motor_velocities_from_payload(const uint8_t* payload, size_t payload_size,
        float out_velocities[MOBILITY_MOTOR_COUNT]);

    bool mobility_unpack_motor_dutycycles_from_payload(const uint8_t* payload, size_t payload_size,
        float out_velocities[MOBILITY_MOTOR_COUNT]);

    bool mobility_unpack_init_packet_from_payload(const uint8_t* payload, size_t payload_size,
        MobilityInitPacket* out_init);

    bool mobility_unpack_enable_from_payload(const uint8_t* payload, size_t payload_size,
        bool* out_enable);

    bool mobility_unpack_device_state_packet_from_payload(const uint8_t* payload, size_t payload_size,
        MobilityDeviceState* out_state);

    bool mobility_unpack_feedback_packet_from_payload(const uint8_t* payload, size_t payload_size,
        MobilityFeedback* out_feedback);



#ifdef __cplusplus
}
#endif

#endif // __MOBILITY_PACKETS_H__