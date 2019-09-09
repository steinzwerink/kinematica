#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include <thread>
#include <string>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib/server/simple_action_server.h>
#include <al5d_pckg/communicatorAction.h>

namespace lld
{
class lld
{
public:
  lld();
  virtual ~lld();
  bool detectDriver();
  void setPort(const std::string port);
  unsigned int angle2Pwm(int aAngle, int istart, int istop, int ostart, int ostop);
  unsigned int remap(int value, int istart, int istop, int ostart, int ostop) const;
  unsigned int mapValues(int aDegree, int aInMin, int aInMax, int aOutMin, int aOutMax) const;
  bool checkRange(const int16_t angle, const uint16_t servo);
  void sendAction(const uint16_t pose, const std::vector<uint16_t> servo, const std::vector<int16_t> angle, const uint16_t duration, const uint16_t size);

private:
  ros::NodeHandle n;
  std::string port;
  uint16_t pose;
  // std::vector<uint16_t> servo;
  // std::vector<int16_t> angle;
  uint16_t duration;
  float value, istart, istop, ostart, ostop;
  const int mSize;
};
} // namespace lld