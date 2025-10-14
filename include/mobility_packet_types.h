#ifndef __MOBILITY_PACKET_TYPES_H__
#define __MOBILITY_PACKET_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

    /* ==============================
     * Constants
     * ============================== */

#define MOBILITY_DEVICE_ID			(0x656B696E)	
#define MOBILITY_MOTOR_COUNT        (4u)

     /* ==============================
      * Commands
      * ============================== */
    typedef enum {
        set_velocity = 0,
        set_dutycycle = 1,
        set_maximum_dutycycle = 2,
        set_lowpass_fc = 3,
        set_pid_params = 4,
        set_pid_bounds = 5,
        set_ff_params = 6,
        set_max_velocity = 7,
        set_feedback_hz = 8,
        set_control_hz = 9,

        open_loop_enable = 10,
        running_mode_enable = 11,
        init_mode_enable = 12,
        
        request_device_state = 32,
        request_motor_feedback = 33,

        init_device = 42,

        motor_feedback = 64,
        device_state = 65

    } MobilityCommands;

    /* ==============================
     * Data Structures
     * ============================== */

#pragma pack(push, 1)
typedef struct {
	uint32_t overwrite_pinout;

	uint8_t lpwm_pins[4];
	uint8_t rpwm_pins[4];

	uint8_t encoder_a_pins[4];

	uint8_t motor_swap_dirs[4];
    uint8_t encoder_swap_dirs[4];

	float max_dutycycle;
	float max_velocity;
	float lowpass_fc;
	float kp;
	float ki;
	float kd;
	float p_bound;
	float i_bound;
    float d_bound;
    
    float feedback_hz;
    float control_hz;
    
    uint8_t is_open_loop;
    
} MobilityInitPacket;
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct {
        uint8_t device_is_init;
        uint8_t device_is_running;
    } MobilityDeviceState;
#pragma pack(pop)


#pragma pack(push, 1)
    typedef struct {
        float positions[MOBILITY_MOTOR_COUNT];
        float velocities[MOBILITY_MOTOR_COUNT];
        float pwm_duties[MOBILITY_MOTOR_COUNT];
    } MobilityFeedback;
#pragma pack(pop)


#pragma pack(push, 1)
    typedef struct {
        uint32_t device_id;
        uint16_t command_id;
        uint16_t sequence;
        uint16_t content_size;
        uint64_t time_since_boot_us;
        uint32_t crc32;
    } MobilityPacketHeader;
#pragma pack(pop)

#pragma pack(push, 1)
    typedef struct {
        MobilityPacketHeader header;
        uint8_t payload[]; /* binary payload, little-endian encoded */
    } MobilityPacket;
#pragma pack(pop)


#ifdef __cplusplus
}
#endif

#endif // __MOBILITY_PACKET_TYPES_H__