#ifndef ROBOTCONTROL_PUBLISHER_HPP
#define ROBOTCONTROL_PUBLISHER_HPP

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <thread>
#include "al5d_pckg/position.h"
#include "al5d_pckg/calibration.h"
#include "al5d_pckg/moveArm.h"

namespace controller
{
class Configuration
{
public:
    Configuration();
    Configuration(std::vector<uint16_t> amServo,
                  std::vector<int16_t> aTheta,
                  uint16_t aDuration);

    std::vector<int16_t> getConfigurationTheta();

    std::vector<uint16_t> getConfigurationServo();

    const uint16_t getConfigurationDuration();

private:
    std::vector<uint16_t> mServo;
    std::vector<int16_t> mTheta;
    uint16_t mDuration;
};

class Controller
{
public:
    Controller(ros::NodeHandle &lN,
               const std::string &lTopic,
               const uint16_t lQueue_size);

    virtual ~Controller() = default;

    void initialize();
    void run();
    void publish(controller::Configuration aConfiguration);

private:
    ros::NodeHandle &mN;
    const std::string &cTopic;
    const uint16_t cQueue_size;
    ros::Publisher mControlPub;
};
} // namespace controller

#endif // ROBOTCONTROL_PUBLISHER_HPP