#include "inc/UdpInterface.h"
#include "inc/armsrov_bridge.h"

ros::NodeHandle* node;

uuv_gazebo_ros_plugins_msgs::FloatStamped thrusters_msg;
ros::Publisher thrusters_pub1;
ros::Publisher thrusters_pub2;
ros::Publisher thrusters_pub3;
ros::Publisher thrusters_pub4;
ros::Publisher thrusters_pub5;
ros::Publisher thrusters_pub6;
ros::Publisher thrusters_pub7;
ros::Publisher thrusters_pub8;

UdpClient client;
MavlinkHandler mavlinkHandler;

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
void pressure_cb(const sensor_msgs::FluidPressure::ConstPtr& msg);
void send_thrusters_input(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4, uint16_t pwm5, uint16_t pwm6,
                          uint16_t pwm7, uint16_t pwm8);
double pwm2rpm(uint16_t pwm);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armsrov_bridge");

  node = new ros::NodeHandle;

  // start UDP server
  boost::asio::io_service io_service;
  UdpServer server(io_service);
  static std::thread run_thread([&] { io_service.run(); });

  ros::Subscriber sub_imu = node->subscribe("/armsrov/imu", 1, imu_cb);
  ros::Subscriber sub_pressure = node->subscribe("/armsrov/pressure", 1, pressure_cb);

  thrusters_pub1 = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsrov/thrusters/1/input", 1);
  thrusters_pub2 = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsrov/thrusters/2/input", 1);
  thrusters_pub3 = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsrov/thrusters/3/input", 1);
  thrusters_pub4 = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsrov/thrusters/4/input", 1);
  thrusters_pub5 = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsrov/thrusters/5/input", 1);
  thrusters_pub6 = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsrov/thrusters/6/input", 1);
  thrusters_pub7 = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsrov/thrusters/7/input", 1);
  thrusters_pub8 = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsrov/thrusters/8/input", 1);

  ros::spin();

  cout << endl << "exiting..." << endl;
  io_service.stop();
  run_thread.join();

  return 0;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  uint32_t time_boot_ms = msg->header.stamp.sec * 1000 + msg->header.stamp.nsec / 1000000;

  double roll, pitch, yaw;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->orientation, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  double rollspeed, pitchspeed, yawspeed;
  rollspeed = msg->angular_velocity.x;
  pitchspeed = msg->angular_velocity.y;
  yawspeed = msg->angular_velocity.z;

  static int count;
  ++count;
  // process data every 2 msg
  if (2 == count)
  {
    count = 0;
    // ROS_INFO("roll: [%f], pitch: [%f], yaw: [%f].", roll, pitch, yaw);
    size_t msg_len =
        mavlinkHandler.attitudeSerialization(time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed);
    client.sendData(mavlinkHandler.attitudeRowData, msg_len);
  }
}

void pressure_cb(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
  uint32_t time_boot_ms = msg->header.stamp.sec * 1000 + msg->header.stamp.nsec / 1000000;

  int32_t alt = (int32_t)((msg->fluid_pressure - 101) / 0.0101);

  // ROS_INFO("alt: %d mm", alt);
  size_t msg_len = mavlinkHandler.altitudeSerialization(time_boot_ms, -alt);
  client.sendData(mavlinkHandler.altitudeRowData, msg_len);
}

void send_thrusters_input(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4, uint16_t pwm5, uint16_t pwm6,
                          uint16_t pwm7, uint16_t pwm8)
{
  static uint32_t seq;

  std_msgs::Header header;
  header.stamp.setNow(ros::Time::now());
  header.frame_id = "base_link";
  header.seq = seq++;

  thrusters_msg.header = header;

  thrusters_msg.data = pwm2rpm(pwm1);
  thrusters_pub1.publish(thrusters_msg);

  thrusters_msg.data = pwm2rpm(pwm2);
  thrusters_pub2.publish(thrusters_msg);

  thrusters_msg.data = pwm2rpm(pwm3);
  thrusters_pub3.publish(thrusters_msg);

  thrusters_msg.data = pwm2rpm(pwm4);
  thrusters_pub4.publish(thrusters_msg);

  thrusters_msg.data = pwm2rpm(pwm5);
  ROS_INFO("rpm5: %f", pwm2rpm(pwm5));
  thrusters_pub5.publish(thrusters_msg);

  thrusters_msg.data = -pwm2rpm(pwm6);
  ROS_INFO("rpm6: %f", -pwm2rpm(pwm6));
  thrusters_pub6.publish(thrusters_msg);

  thrusters_msg.data = -pwm2rpm(pwm7);
  ROS_INFO("rpm7: %f", -pwm2rpm(pwm7));
  thrusters_pub7.publish(thrusters_msg);

  thrusters_msg.data = pwm2rpm(pwm8);
  ROS_INFO("rpm8: %f", pwm2rpm(pwm8));
  thrusters_pub8.publish(thrusters_msg);
}

double pwm2rpm(uint16_t pwm)
{
  double ret;
  if (pwm > 1500)
  {
    ret = sqrt((double)(pwm - 1500)) * 400;
  }
  else if (pwm < 1500)
  {
    ret = -sqrt((double)(1500 - pwm)) * 400;
  }
  else
  {
    ret = 0;
  }

  return ret;
}
