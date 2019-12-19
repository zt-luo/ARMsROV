#include "inc/MavlinkHandler.h"

MavlinkHandler::MavlinkHandler(uint8_t sys_id)
  : _channel(MAVLINK_COMM_0), _mavlink_system_id(sys_id), _vehicle_arm(false), _vehicle_mode(MANUAL)
{
  attitudeRowData = new char[_raw_data_max_bytes];
  altitudeRowData = new char[_raw_data_max_bytes];
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
  {  // start msg_received

    // handle message
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

      if (rc.target_system != _mavlink_system_id)
      {
        // not for me
        return;
      }

      if (!_vehicle_arm)
      {
        ROS_INFO("ARM first!");
        return;
      }

      if (_vehicle_mode != LAB_REMOTE)
      {
        ROS_INFO("NOT in LAB_REMOTE mode. set mode first.");
        return;
      }

      send_thrusters_input(rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw, rc.chan5_raw, rc.chan6_raw,
                           rc.chan7_raw, rc.chan8_raw);
      // ROS_INFO("RC_CHANNELS_OVERRIDE: %d %d %d %d %d %d %d %d.", rc.chan1_raw, rc.chan2_raw, rc.chan3_raw,
      // rc.chan4_raw,
      //          rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw);
      // ROS_INFO("RC_CHANNELS_OVERRIDE");
    }
    else if (message.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL)
    {
      // ROS_INFO("MAVLINK_MSG_ID_MANUAL_CONTROL");
      mavlink_manual_control_t mc;
      mavlink_msg_manual_control_decode(&message, &mc);

      if (mc.target != _mavlink_system_id)
      {
        // not for me
        return;
      }

      applyManualControl(&mc);

      ROS_INFO("MANUAL: x: %d, y: %d, z: %d, r: %d, buttons: %d.", mc.x, mc.y, mc.z, mc.r, mc.buttons);
    }
    else if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG)
    {
      mavlink_command_long_t cl;
      mavlink_msg_command_long_decode(&message, &cl);

      if (cl.target_system != _mavlink_system_id)
      {
        // not for me
        return;
      }

      // handle arm disarm
      if (cl.command == MAV_CMD_COMPONENT_ARM_DISARM)
      {
        if (cl.param1 == 1.0)
        {
          if (_vehicle_arm)
          {
            ROS_INFO("already ARM!");
          }
          else
          {
            _vehicle_arm = true;
            ROS_INFO("vehicle ARM!");
          }
        }
        else
        {
          if (_vehicle_arm)
          {
            _vehicle_arm = false;
            send_thrusters_input(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500);
            ROS_INFO("vehicle DISARM!");
          }
          else
          {
            ROS_INFO("already DISARM!");
          }
        }
      }
      else
      {
        ROS_INFO("unknown command: %d.", cl.command);
      }
    }
    else if (message.msgid == MAVLINK_MSG_ID_SET_MODE)
    {
      // handle set mode
      mavlink_set_mode_t sm;
      mavlink_msg_set_mode_decode(&message, &sm);

      if (sm.target_system != _mavlink_system_id)
      {
        // not for me
        return;
      }

      _vehicle_mode = (control_mode_t)(sm.custom_mode);

      if ((control_mode_t)sm.custom_mode == LAB_REMOTE)
      {
        ROS_INFO("set vehicle mode to: LAB_REMOTE");
      }
      else if ((control_mode_t)sm.custom_mode == MANUAL)
      {
        ROS_INFO("set vehicle mode to: MANUAL");
      }
      else
      {
        ROS_INFO("set vehicle mode to: %d (NOT supported).", sm.custom_mode);
      }
    }
    else
    {
      ROS_INFO("unknown msgid: %d.", message.msgid);
    }
  }  // start msg_received
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
  mavlink_msg_attitude_encode(_mavlink_system_id, _mavlink_component_id, &message, &mavlink_attitude);

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
  mavlink_msg_global_position_int_encode(_mavlink_system_id, _mavlink_component_id, &message,
                                         &mavlink_global_position_int);

  int msg_len = mavlink_msg_to_send_buffer((uint8_t *)altitudeRowData, &message);
  return msg_len;
}

void MavlinkHandler::applyManualControl(mavlink_manual_control_t *mc)
{
  int x = mc->x;
  int y = mc->y;
  int depth = mc->z - 500;
  int yaw = mc->r;
  int pitch = 0;
  int roll = 0;

  x *= 0.1;
  y *= 0.1;
  depth *= -0.1;
  yaw *= 0.1;
  pitch *= 0.25;
  roll *= 0.25;

  int motor_pwm[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

  motor_pwm[0] = 1500 + yaw + x + y;
  motor_pwm[1] = 1500 - yaw + x - y;
  motor_pwm[2] = 1500 - yaw - x + y;
  motor_pwm[3] = 1500 + yaw - x - y;

  motor_pwm[4] = 1500 + depth - pitch + roll;
  motor_pwm[5] = 1500 - depth + pitch + roll;
  motor_pwm[6] = 1500 - depth - pitch - roll;
  motor_pwm[7] = 1500 + depth + pitch - roll;

  send_thrusters_input(motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3], motor_pwm[4], motor_pwm[5], motor_pwm[6],
                       motor_pwm[7]);
}
