#include "al5d_pckg/Controller.h"

controller::Configuration::Configuration() : mServo({1, 2, 3}), mTheta({40, 50, -65}), mDuration(2000)
{
}

controller::Configuration::Configuration(std::vector<uint16_t> amServo,
                                         std::vector<int16_t> aTheta,
                                         uint16_t aDuration) : mServo(amServo), mTheta(aTheta), mDuration(aDuration) {}

controller::Controller::Controller(ros::NodeHandle &lN,
                                   const std::string &lTopic,
                                   const uint16_t lQueue_size)
    : mN(lN),
      cTopic(lTopic),
      cQueue_size(lQueue_size),
      mControlPub(
          mN.advertise<al5d_pckg::moveArm>(cTopic, cQueue_size))
{
}

void controller::Controller::publish(controller::Configuration aConfiguration)
{
    al5d_pckg::moveArm lMsg;
    lMsg.angle.resize(aConfiguration.getConfigurationTheta().size());
    lMsg.angle = aConfiguration.getConfigurationTheta();

    lMsg.servo.resize(aConfiguration.getConfigurationServo().size());
    lMsg.servo = aConfiguration.getConfigurationServo();

    lMsg.duration = aConfiguration.getConfigurationDuration();

    mControlPub.publish(lMsg);
}

void controller::Controller::run()
{
}

std::vector<int16_t> controller::Configuration::getConfigurationTheta()
{
    return mTheta;
}

std::vector<uint16_t> controller::Configuration::getConfigurationServo()
{
    return mServo;
}

const uint16_t controller::Configuration::getConfigurationDuration()
{
    return mDuration;
}