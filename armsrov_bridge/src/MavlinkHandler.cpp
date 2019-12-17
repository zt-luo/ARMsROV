#include "inc/MavlinkHandler.h"

MavlinkHandler::MavlinkHandler(uint8_t sys_id) : _channel(MAVLINK_COMM_0), mavlink_system_id(sys_id)
{
  attitudeRowData = new char[raw_data_max_bytes];
  altitudeRowData = new char[raw_data_max_bytes];
}

MavlinkHandler::~MavlinkHandler()
{
  delete[] attitudeRowData;
  delete[] altitudeRowData;
}

void MavlinkHandler::parseChars(char *c, size_t len)
{
  mavlink_message_t message;
  mavlink_status_t status;
  bool msg_received = false;

  for (size_t i = 0; i < len; i++)
  {
    msg_received = mavlink_parse_char(_channel, c[i], &message, &status);
  }

  if (true == msg_received)
  {
    //  handle message
    if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
      mavlink_heartbeat_t hb;
      mavlink_msg_heartbeat_decode(&message, &hb);

      ROS_INFO("mavlink heartbeat received, from ID: %d.", message.sysid);
    }
    else if (message.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE)
    {
      mavlink_rc_channels_override_t rc;
      mavlink_msg_rc_channels_override_decode(&message, &rc);

      send_thrusters_input(rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw, rc.chan5_raw, rc.chan6_raw,
                           rc.chan7_raw, rc.chan8_raw);
      // ROS_INFO("RC_CHANNELS_OVERRIDE: %d %d %d %d %d %d %d %d.", rc.chan1_raw, rc.chan2_raw, rc.chan3_raw,
      // rc.chan4_raw,
      //          rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw);
      // ROS_INFO("RC_CHANNELS_OVERRIDE");
    }
    else if (message.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL)
    {
      ROS_INFO("MAVLINK_MSG_ID_MANUAL_CONTROL");
    }
  }
}

size_t MavlinkHandler::attitudeSerialization(uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed,
                                             float pitchspeed, float yawspeed)
{
  mavlink_attitude_t mavlink_attitude;
  mavlink_attitude.time_boot_ms = time_boot_ms;
  mavlink_attitude.roll = roll;
  mavlink_attitude.pitch = pitch;
  mavlink_attitude.yaw = yaw;
  mavlink_attitude.rollspeed = rollspeed;
  mavlink_attitude.pitchspeed = pitchspeed;
  mavlink_attitude.yawspeed = yawspeed;

  mavlink_message_t message;
  mavlink_msg_attitude_encode(mavlink_system_id, mavlink_component_id, &message, &mavlink_attitude);

  int msg_len = mavlink_msg_to_send_buffer((uint8_t *)attitudeRowData, &message);

  return msg_len;
}

size_t MavlinkHandler::altitudeSerialization(uint32_t time_boot_ms, int32_t alt)
{
  mavlink_global_position_int_t mavlink_global_position_int;
  mavlink_global_position_int.time_boot_ms = time_boot_ms;
  mavlink_global_position_int.alt = alt;
  mavlink_global_position_int.hdg = 0;
  mavlink_global_position_int.lat = 0;
  mavlink_global_position_int.lon = 0;
  mavlink_global_position_int.relative_alt = 0;
  mavlink_global_position_int.vx = 0;
  mavlink_global_position_int.vy = 0;
  mavlink_global_position_int.vz = 0;

  mavlink_message_t message;
  mavlink_msg_global_position_int_encode(mavlink_system_id, mavlink_component_id, &message,
                                         &mavlink_global_position_int);

  int msg_len = mavlink_msg_to_send_buffer((uint8_t *)altitudeRowData, &message);
  return msg_len;
}
