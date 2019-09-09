
#include "al5d_pckg/MatrixCalculations.h"
#include <vector>
namespace al5d
{

const int16_t cMax = 2500;
const int16_t cMin = 500;
const int16_t cMaxDegrees = 90;
const int16_t cMinDegrees = -90;

std::vector<int16_t> cPwmComp={-50,30, 340,10, 0, 111};

const int16_t cGripperOpenDegrees= 60;
const int16_t cGripperClosedDegrees= 180;
const int16_t cGripperOpenCm= 3;
const int16_t cGripperClosedCm= 0;
const int16_t cGripperOpen= 60;
const int16_t cGripperCorrection= 40;

const int16_t cAngleCorrection= 5;

const int16_t cAboveGround=-4;
const int16_t cOnGround=-6;

// (0 = shoulder to elbow, 1 = elbow to wrist, 2 = wrist to gripper)
Matrix<double, 3, 1> cSidelengths = {{{14.6}},
                                     {{18.7}},
                                     {{10.0}}};



Matrix<double, 3, 2> cThetaRanges = {{{-30}, {90}}, {{0}, {135}}, {{-90}, {90}}};

std::vector<double> cServos = {
    -100,
    100,
    -90,
    90,
    0,
    220,
    -90,
    90,
    0,
    180,
    -90,
    90,
};

} // namespace al5d
