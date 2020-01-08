#pragma once

#include <ros/ros.h>

#include "../../mavlink_c_library_v2/ardupilotmega/mavlink.h"

extern void send_thrusters_input(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4, uint16_t pwm5,
                                 uint16_t pwm6, uint16_t pwm7, uint16_t pwm8);

// Auto Pilot Modes enumeration
typedef enum control_mode_enum
{
  STABILIZE = 0,     // manual angle with manual depth/throttle
  ACRO = 1,          // manual body-frame angular rate with manual depth/throttle
  ALT_HOLD = 2,      // manual angle with automatic depth/throttle
  AUTO = 3,          // fully automatic waypoint control using mission commands
  GUIDED = 4,        // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
  CIRCLE = 7,        // automatic circular flight with automatic throttle
  SURFACE = 9,       // automatically return to surface, pilot maintains horizontal control
  POSHOLD = 16,      // automatic position hold with manual override, with automatic throttle
  MANUAL = 19,       // Pass-through input with no stabilization
  LAB_REMOTE = 60,   // lab mode
  LAB_ALT_HOLD = 61  // lab alt_hold mode
} control_mode_t;

class MavlinkHandler
{
private:
  mavlink_channel_t _channel;
  const int _raw_data_max_bytes = 279;
  const int _mavlink_component_id = 1;

  int _mavlink_system_id;
  bool _vehicle_arm;
  control_mode_t _vehicle_mode;

  void applyManualControl(mavlink_manual_control_t* mc);

public:
  MavlinkHandler(uint8_t sys_id);
  ~MavlinkHandler();

  char* attitudeRowData;
  char* altitudeRowData;

  void parseChars(char* c, size_t len);
  void handelMessage(mavlink_message_t message);
  size_t attitudeSerialization(uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed,
                               float pitchspeed, float yawspeed);
  size_t altitudeSerialization(uint32_t time_boot_ms, int32_t alt);
};
