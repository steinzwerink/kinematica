#ifndef MatrixCalculations_H
#define MatrixCalculations_H

#include <math.h>
#include <random>

#include "al5d_pckg/Matrix.hpp"

#define PI 3.14159265
namespace MatrixCalculations
{
/**
   * @brief Construct a new Jacobi matrix n
   * @param aArmlength length of robot arm
   * @param athetas Angles of every arm
   * @return A 2 * 3 sized Jacobi-matrix
   */
Matrix<double, 2, 3> computeJacobi(Matrix<double, 3, 1> aArmlength, Matrix<double, 3, 1> aThetas);

/**
   * @brief Computes end effector for a robot in 2D x,y. 
   * @param aArmlength - length of robot arm
   * @param aThetas - Angles of every arm
   * @return Matrix containing X and Y of end effector [0][0] contains x, [0][1] contains y
   */
Matrix<double, 2, 1> computeEndEffector(Matrix<double, 3, 1> aArmlength, Matrix<double, 3, 1> aThetas);

/**
   * @brief Converts a given angle into radians
   * @return radians. 
   */
double degreesToRadians(const double aAngle);

/**
   * @brief Computes the inverse of a jacobi matrix,
   * @return inverse maxtrix
   */
Matrix<double, 3, 2> computeInverseJacobi(const Matrix<double, 2, 3> &aOriginalJacobi);

/**
   * @brief Checks whether a set of theta's falls within the allowed ranges.
   * @param aThetas - Three different theta's in degrees
   * @param aThetaRanges - Three different pairs of {min, max} angles in degrees.
   * @return Returns true if the given theta's fall within the ranges.
   */
bool CheckThetasInRange(Matrix<double, 3, 1> &aThetas, Matrix<double, 3, 2> &aThetaRanges);

/**
   * @brief Randomizes a set of given theta's to values within a given range
   * @param aThetas - The given theta's.
   * @param aThetaRanges - Three different pairs of {min, max} angles in degrees
   */
void genRandThetas(Matrix<double, 3, 1> &aThetas, Matrix<double, 3, 2> &aThetaRanges);

/**
   * @brief Inverse kinematics, attempts to find a (valid) configuration for a certain x,y position.  
   * @param aGoal - The goal: [0][0] = X and [1][0] = Y.
   * @param aSidelength -  lengths of robot arm
   * 
   * @param aCurrentThetas - Current configuration of theta's
   * @param aThetaRanges - The allowed theta ranges for the configuration to be considered valid. 
   * @param aIterations - The max number of attempts. 
   * @return A pair, first element of pair indicates whether a valid configuration was found. Second element is the configuration
   */
std::pair<bool, Matrix<double, 3, 1>> computeConfiguration(Matrix<double, 2, 1> &aGoal, const Matrix<double, 3, 1> &aArmlength, const Matrix<double, 3, 1> &aCurrentThetas, Matrix<double, 3, 2> &aThetaRanges, int aIterations);

/**
   * @brief Calculates the angle the goal has to the base.
   * @param aBase - Base point x,y
   * @param aTarget - Target point x,y 
   * @return double - Angle, when target is below the X-axis of the base this angle is negative, because -90to 90 range
   */
Matrix<double, 2, 1> getBasePoint(Matrix<double, 2, 1> aPos);

double calculateAngle(Matrix<double, 2, 1> aP1, Matrix<double, 2, 1> aP2);

/**
   * @brief Calculates the minimum distance between two points
   * @param aPoint1 - First point 
   * @param aPoint2 - Second point
   * @return The distance between both points
   */
double getDistance(Matrix<double, 2, 1> aPoint1, Matrix<double, 2, 1> aPoint2);

unsigned int remap(int value, int istart, int istop, int ostart, int ostop) ;

}; // namespace MatrixCalculations

#endif // MatrixCalculations.h
