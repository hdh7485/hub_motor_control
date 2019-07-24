#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <string>

typedef uint8_t BYTE;

struct IBYTE {
  BYTE byLow;
  BYTE byHigh;
};

class HubMotorControl {
private:
  ros::NodeHandle nh;
  ros::NodeHandle nh_;
  ros::Subscriber rpm_sub_;

  serial::Serial hub_motor_serial_;
  std::string port_;
  int baud_;
  int timeout_;
  bool reverse_x_direction_;

public:
  HubMotorControl() {
    nh_ = ros::NodeHandle("~");
    nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
    nh_.param("baudrate", baud_, 19200);
    nh_.param("timeout", timeout_, 1000);

    nh_.param("reverse_x_direction", reverse_x_direction_, false);

    rpm_sub_ = nh.subscribe<std_msgs::Int16> ("pwm", 10, &HubMotorControl::rpmCallback, this);

    hub_motor_serial_.setPort(port_);
    hub_motor_serial_.setBaudrate(baud_);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_);
    hub_motor_serial_.setTimeout(timeout);

    try {
      hub_motor_serial_.open();
    }
    catch(serial::IOException ex) {
      ROS_ERROR("Serial Port Closed. Turn off the node.");
      exit(-1);
    }
  }

  BYTE getCheckSum(short nPacketSize, BYTE *byArray) {
    BYTE byTmp = 0;
    short i;

    for (i = 0; i < nPacketSize; i++) {
      byTmp += *(byArray + i);
    }
    return (~byTmp + 1);
  }

  IBYTE int2byte (short nln) {
    IBYTE ibyte;
    ibyte.byLow = nln & 0xff;
    ibyte.byHigh = nln>>8 & 0xff;
    return ibyte;
  }

  void rpmCallback(const std_msgs::Int16::ConstPtr& rpm_msg) {
    IBYTE rpm = int2byte(rpm_msg->data);

    BYTE rpm_packet[8] = {0,};
    rpm_packet[0] = 183;
    rpm_packet[1] = 184;
    rpm_packet[2] = 1;
    rpm_packet[3] = 130;
    rpm_packet[4] = 2;
    rpm_packet[5] = rpm.byLow;
    rpm_packet[6] = rpm.byHigh;
    BYTE checksum = getCheckSum(7, rpm_packet);
    rpm_packet[7] = checksum;
    
    //std::string command_string = rpm_packet;

    hub_motor_serial_.write(rpm_packet, 8);
    for (int i = 0; i < 8; i++) ROS_INFO("%d", rpm_packet[i]);
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dr_control");
  HubMotorControl hub_motor_control;

  ros::spin();

  return 0;
}
