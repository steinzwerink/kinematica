#include "al5d_pckg/Programhandler.hpp"
#include "al5d_pckg/position.h"
#include "al5d_pckg/MatrixCalculations.h"
#include <thread>
#include "ros/ros.h"
class MainApplication
{

public:
    MainApplication();

    ~MainApplication() = default;

    void camRecord();

    void initialize(int aSampleSize);

    void run();

    void parseArguments(int argc, char **argv);

    const std::pair<double, double> getXYCube();

    const std::pair<double, double> getXYDropZone();

    const double getBase2Goal();

    const double getBase2Drop();

    const cv::Point2f getGoal();

    const double getBase2GoalAngle();

    const double getShapeAngle();

    const double getWidth();

private:
    cv::Mat mImage, mInitializeImage;
    cv::VideoCapture cap; // open the default camera
    bool input = false;
    int coutValue;
    Programhandler P;
    al5d_pckg::position mMsg;
    ros::NodeHandle n;
    float mPixelsPerCm;
    cv::Scalar mCsqr;
    Matrix<double, 2, 1> mCubePos;
    double mXCube;
    double mYCube;
    double mXdropPoint;
    double mYdropPoint;
    double mBase2GoalAngle;
    double mBase2drop;
    double mBase2goal;
    double mShapeAngle;
    double mWidth;
    Matrix<double, 2, 1> mBasePos;
    Matrix<double, 2, 1>
        mDropPos;
    cv::Point2f mGoal;
    int fpc;
    std::string mColor_s, mShape_s;
};