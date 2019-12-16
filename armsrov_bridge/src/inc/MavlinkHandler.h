#pragma once

#include <ros/ros.h>

#include "../../mavlink_c_library_v2/ardupilotmega/mavlink.h"

extern void send_thrusters_input(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4, uint16_t pwm5,
                                 uint16_t pwm6, uint16_t pwm7, uint16_t pwm8);

#define ROW_DATA_MAX_BYTES (279)
#define MAVLINK_SYSTEM_ID (121)
#define MAVLINK_COMPONENT_ID (1)

class MavlinkHandler
{
private:
  mavlink_channel_t _channel;

public:
  MavlinkHandler();
  ~MavlinkHandler();

  char* attitudeRowData;
  char* altitudeRowData;

  void parseChars(char* c, size_t len);
  size_t attitudeSerialization(uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed,
                               float pitchspeed, float yawspeed);
  size_t altitudeSerialization(uint32_t time_boot_ms, int32_t alt);
};
