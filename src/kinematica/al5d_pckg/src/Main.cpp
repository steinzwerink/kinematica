#include <ros/ros.h>
#include "al5d_pckg/ConstValues.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <al5d_pckg/communicatorAction.h>
#include "al5d_pckg/client.h"
#include "al5d_pckg/Controller.h"
#include "al5d_pckg/MainApplication.hpp"

//messages
#include "al5d_pckg/position.h"

#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "AL5D");

    ros::NodeHandle lControlNode;
    ros::Rate lLoop_rate(1);
    controller::Controller lControllerPub(lControlNode, "/moveServo", 1000);
    // client::client c;
    MainApplication M;
    cv::namedWindow("TEST", CV_WINDOW_AUTOSIZE);
    cv::resizeWindow("TEST", 800, 800);

    M.initialize(10);
    M.run();

    double lGripperAngle = M.getBase2GoalAngle() - M.getShapeAngle();
    if (lGripperAngle > 90.0)
        lGripperAngle -= 180;
    double lGripperValue = MatrixCalculations::remap(M.getWidth(), al5d::cGripperClosedCm, al5d::cGripperOpenCm, al5d::cGripperClosedDegrees, al5d::cGripperOpenDegrees);
    lGripperValue += al5d::cGripperCorrection;
    double lCorrectedAngle = M.getBase2GoalAngle();
    if (M.getBase2GoalAngle() > 0)
        lCorrectedAngle -= al5d::cAngleCorrection;

    Matrix<double, 2, 1> lAboveGoalEffector = {{{M.getBase2Goal() + 2}}, {{al5d::cAboveGround}}};
    Matrix<double, 3, 1> mCurrentThetas = {0, 0, 0};
    std::vector<std::pair<bool, Matrix<double, 3, 1>>> lConfigurations;

    std::pair<bool, Matrix<double, 3, 1>> lConfiguration = MatrixCalculations::computeConfiguration(lAboveGoalEffector, al5d::cSidelengths, mCurrentThetas, al5d::cThetaRanges, 500);
    lConfigurations.push_back(lConfiguration);
    mCurrentThetas = lConfiguration.second;

    Matrix<double, 2, 1> lGoalEffector = {{{M.getBase2Goal() + 2}}, {{al5d::cOnGround}}};

    lConfiguration = MatrixCalculations::computeConfiguration(lGoalEffector, al5d::cSidelengths, mCurrentThetas, al5d::cThetaRanges, 500);
    lConfigurations.push_back(lConfiguration);
    mCurrentThetas = lConfiguration.second;

    Matrix<double, 2, 1> lAboveDropEffector = {{{M.getBase2Drop()}}, {{al5d::cAboveGround}}};
    lConfiguration = MatrixCalculations::computeConfiguration(lAboveDropEffector, al5d::cSidelengths, mCurrentThetas, al5d::cThetaRanges, 500);
    lConfigurations.push_back(lConfiguration);
    mCurrentThetas = lConfiguration.second;

    for (size_t lIndex = 0; lIndex < lConfigurations.size(); ++lIndex)
    {
        if (lConfigurations.at(lIndex).first == true)
            std::cout << "configuration found:" << lConfigurations.at(lIndex).second << std::endl;
        else
            std::cout << "No configuration found" << std::endl;
    }

    controller::Configuration lAngleGoal({0, 5}, {static_cast<int16_t>(lCorrectedAngle), static_cast<int16_t>(lGripperAngle)}, 2000);

    controller ::Configuration lAboveGoal({1, 2, 3}, {static_cast<int16_t>(lConfigurations.at(0).second[0][0] * -1), static_cast<int16_t>(lConfigurations.at(0).second[1][0]), static_cast<int16_t>((lConfigurations.at(0).second[2][0] * -1))}, 4000);

    controller ::Configuration lGoal({1, 2, 3}, {static_cast<int16_t>(lConfigurations.at(1).second[0][0] * -1), static_cast<int16_t>(lConfigurations.at(1).second[1][0]), static_cast<int16_t>(lConfigurations.at(1).second[2][0] * -1)},
                                     2000);

    controller::Configuration lCloseGripperAngle({4}, {static_cast<int16_t>(lGripperValue)}, 2000);

    controller ::Configuration lAboveDrop({1, 2, 3}, {static_cast<int16_t>(lConfigurations.at(2).second[0][0] * -1), static_cast<int16_t>(lConfigurations.at(2).second[1][0]), static_cast<int16_t>(lConfigurations.at(2).second[2][0] * -1)},
                                          2000);

    controller ::Configuration lDrop({0, 5}, {0, 0}, 1000);

    controller::Configuration lOpenGripperAngle({4}, {60}, 2000);

    controller::Configuration lStraight({0, 1, 2, 3, 4}, {0, 0, 0, 0, 60}, 2000);

    std::vector<controller::Configuration> lStartUpSequence = {
        lStraight,
        lAngleGoal,
        lAboveGoal,
        lGoal,
        lCloseGripperAngle,
        lAboveDrop,
        lDrop,
        lOpenGripperAngle,
        lStraight};

    for (auto &f : lStartUpSequence)
    {
        lControllerPub.publish(f);
        ros::Duration(f.getConfigurationDuration() / 1000).sleep();
    }

    ros::spinOnce();
    lLoop_rate.sleep();

    return 0;
}
