#include "al5d_pckg/MatrixCalculations.h"

Matrix<double, 2, 3> MatrixCalculations::computeJacobi(Matrix<double, 3, 1> aArmlength, Matrix<double, 3, 1> aThetas)
{
    Matrix<double, 2, 3> lJacobi;
    std::vector<double> lThetas;
    double lTheta1, lTheta2, lTheta3, lSide1, lSide2, lSide3;
    lTheta1 = degreesToRadians(aThetas[0][0]);
    lTheta2 = degreesToRadians(aThetas[1][0]);
    lTheta3 = degreesToRadians(aThetas[2][0]);

    lSide1 = aArmlength[0][0];
    lSide2 = aArmlength[1][0];
    lSide3 = aArmlength[2][0];

    lJacobi[0][0] = (lSide1 * cos(lTheta1)) + (lSide2 * cos(lTheta1 + lTheta2)) + (lSide3 * cos(lTheta1 + lTheta2 + lTheta3));
    lJacobi[0][1] = (lSide2 * cos(lTheta1 + lTheta2)) + (lSide3 * cos(lTheta1 + lTheta2 + lTheta3));
    lJacobi[0][2] = (lSide3 * cos(lTheta1 + lTheta2 + lTheta3));

    lJacobi[1][0] = -(lSide1 * sin(lTheta1)) - (lSide2 * sin(lTheta1 + lTheta2)) - (lSide3 * sin(lTheta1 + lTheta2 + lTheta3));
    lJacobi[1][1] = -(lSide2 * sin(lTheta1 + lTheta2)) - (lSide3 * sin(lTheta1 + lTheta2 + lTheta3));
    lJacobi[1][2] = -(lSide3 * sin(lTheta1 + lTheta2 + lTheta3));

    return lJacobi;
}

Matrix<double, 2, 1> MatrixCalculations::computeEndEffector(Matrix<double, 3, 1> aArmlength, Matrix<double, 3, 1> aThetas)
{
    Matrix<double, 2, 1> lEndEffector;
    double lTheta1, lTheta2, lTheta3;
    lTheta1 = degreesToRadians(aThetas[0][0]);
    lTheta2 = degreesToRadians(aThetas[1][0]);
    lTheta3 = degreesToRadians(aThetas[2][0]);

    lEndEffector[0][0] = (aArmlength[0][0] * sin(lTheta1)) + (aArmlength[1][0] * sin(lTheta1 + lTheta2)) + (aArmlength[2][0] * sin(lTheta1 + lTheta2 + lTheta3));

    lEndEffector[0][1] = (aArmlength[0][0] * cos(lTheta1)) + (aArmlength[1][0] * cos(lTheta1 + lTheta2)) + (aArmlength[2][0] * cos(lTheta1 + lTheta2 + lTheta3));

    return lEndEffector;
}

double MatrixCalculations::degreesToRadians(const double aAngle)
{
    return aAngle * (M_PI / 180.0);
}

Matrix<double, 3, 2> MatrixCalculations::computeInverseJacobi(const Matrix<double, 2, 3> &aJacobi)
{
    return (aJacobi.transpose() * (aJacobi * aJacobi.transpose()).inverse());
}

bool MatrixCalculations::CheckThetasInRange(Matrix<double, 3, 1> &aThetas, Matrix<double, 3, 2> &aThetaRanges)
{
    bool lCheck = true;
    for (size_t i = 0; i < aThetas.getRows(); ++i)
    {
        if (!(aThetas[i][0] >= aThetaRanges[i][0] && aThetas[i][0] <= aThetaRanges[i][1]))
            lCheck = false;
    }

    return lCheck;
}

void MatrixCalculations::genRandThetas(Matrix<double, 3, 1> &aThetas, Matrix<double, 3, 2> &aThetaRanges)
{
    for (size_t i = 0; i < aThetas.getRows(); ++i)
    {
        aThetas[i][0] = (aThetaRanges[i][1] - aThetaRanges[i][0]) * ((double)rand() / (double)RAND_MAX) + aThetaRanges[i][0];
    }
}

Matrix<double, 2, 1> MatrixCalculations::getBasePoint(Matrix<double, 2, 1> aPos)
{
    aPos[0][0] += 7;
    return aPos;
}

double MatrixCalculations::calculateAngle(Matrix<double, 2, 1> aP1, Matrix<double, 2, 1> aP2)
{
    double angle = atan2(aP1[1][0] - aP2[1][0], aP1[0][0] - aP2[0][0]);
    return angle * 180 / PI;
}

double MatrixCalculations::getDistance(Matrix<double, 2, 1> aPoint1, Matrix<double, 2, 1> aPoint2)
{
    return sqrt(pow(aPoint1[0][0] - aPoint2[0][0], 2) + pow(aPoint1[1][0] - aPoint2[1][0], 2));
}

unsigned int MatrixCalculations::remap(int value, int istart, int istop, int ostart, int ostop)
{
    return (ostop - ostart) * (value - istart) / (istop - istart) + ostart;
}

std::pair<bool, Matrix<double, 3, 1>> MatrixCalculations::computeConfiguration(Matrix<double, 2, 1> &aGoal, const Matrix<double, 3, 1> &aArmlength, const Matrix<double, 3, 1> &aCurrentThetas, Matrix<double, 3, 2> &aThetaRanges, int aIterations)
{
    std::pair<bool, Matrix<double, 3, 1>> lComputedConfig;
    bool lFoundConfig = false;
    Matrix<double, 3, 1> lCurrentConfiguration = aCurrentThetas;
    double lDeltaSize = 0.1;
    int lIterations = 0;

    Matrix<double, 2, 1> lEndEffector = MatrixCalculations::computeEndEffector(aArmlength, lCurrentConfiguration);

    while (!lFoundConfig && lIterations < aIterations)
    {
        while (!equals(lEndEffector, aGoal, 0.05))
        {

            Matrix<double, 2, 3> lOriginalJacobi = MatrixCalculations::computeJacobi(aArmlength, lCurrentConfiguration);

            Matrix<double, 3, 2> inverseJacobi = MatrixCalculations::computeInverseJacobi(lOriginalJacobi);

            Matrix<double, 2, 1> deltaEffector = (aGoal - lEndEffector) * lDeltaSize;

            Matrix<double, 3, 1> deltaThetas = inverseJacobi * deltaEffector;

            lCurrentConfiguration += deltaThetas;

            lEndEffector = MatrixCalculations::computeEndEffector(aArmlength, lCurrentConfiguration);
        }

        lIterations++;

        if (MatrixCalculations::CheckThetasInRange(lCurrentConfiguration, aThetaRanges) && lCurrentConfiguration[2][0] >= 0)
            lFoundConfig = true;
        else
        {
            MatrixCalculations::genRandThetas(lCurrentConfiguration, aThetaRanges);
            lEndEffector = MatrixCalculations::computeEndEffector(aArmlength, lCurrentConfiguration);
        }
    }

    lComputedConfig.first = lFoundConfig;
    lComputedConfig.second = lCurrentConfiguration;

    return lComputedConfig;
}