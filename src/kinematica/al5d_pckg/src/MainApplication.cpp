#include "al5d_pckg/MainApplication.hpp"
#include <math.h> /* atan */

MainApplication::MainApplication() : cap(0), P("spec.batch", interactive_modus), mCsqr(255, 255, 255), mColor_s("red"), mShape_s("calibrate"), fpc(10)
{
}

void MainApplication::camRecord()
{
	cap.read(mImage);
	cv::imwrite("../original.jpg", mImage);
	//mImage = cv::imread("/home/stein/Desktop/Herkansing_opencv/src/open_cv/include/2.jpg", CV_LOAD_IMAGE_COLOR);
}

void MainApplication::initialize(int aSampleSize)
{
	cv::Mat lImage;
	for (int i = 0; i < aSampleSize; ++i)
	{
		std::pair<double, double> lCubeTotal = {0, 0};
		double lTotal = 0;

		int lFaultyMeasurements = 0;
		P.setStartValues(mColor_s);
		for (int i = 0; i < fpc; ++i)
		{
			camRecord();
			lImage = mImage;
			mInitializeImage = P.run_program(mColor_s, mShape_s, lImage, coutValue); // get pixels per cm

			if (P.mPixelsPerCm > 15 && P.mPixelsPerCm < 25.5)
			{
				lCubeTotal.first += P.mCoordinates.first;
				lCubeTotal.second += P.mCoordinates.second;
				lTotal += P.mPixelsPerCm;
			}
			else
				++lFaultyMeasurements;
		} //calibration values

		mPixelsPerCm = lTotal / (fpc - lFaultyMeasurements);
		mXCube = lCubeTotal.first / (fpc - lFaultyMeasurements);
		mYCube = lCubeTotal.second / (fpc - lFaultyMeasurements);

		if (mPixelsPerCm > 15 && mPixelsPerCm < 25.5)
		{
			std::cout
				<< "mPixelsPerCm:" << mPixelsPerCm << std::endl;
		}
		else
		{
			//	throw std::logic_error("mPixelsPerCm preconditions are not met,  must be smaller then 19 and bigger then 18 for valid calculations");
		}

		Matrix<double, 2, 1>
			lCubePos = {{{mXCube / mPixelsPerCm}}, {{mYCube / mPixelsPerCm}}};
		mCubePos += lCubePos;
	}
	std::cout << "cube x: " << mCubePos[0][0] / aSampleSize << "y: " << mCubePos[1][0] / aSampleSize << std::endl;

	//found objects
	mBasePos = MatrixCalculations::getBasePoint((mCubePos / aSampleSize));

	std::cout << "base: " << mBasePos[1][0] << std::endl;
	mColor_s = "white";
	mShape_s = "circle";
	P.setStartValues(mColor_s);
	mInitializeImage = P.run_program(mColor_s, mShape_s, lImage, coutValue); // get x and y pos of the drop off zone

	mXdropPoint = P.mCoordinates.first / mPixelsPerCm;
	mYdropPoint = P.mCoordinates.second / mPixelsPerCm;

	std::ostringstream lDropPoint;

	lDropPoint << std::fixed;
	lDropPoint << std::setprecision(0);
	lDropPoint << "(" << mXdropPoint << "," << mYdropPoint << ")";

	std::string lDropPointStr = lDropPoint.str();

	cv::putText(mInitializeImage, lDropPointStr, cv::Point(30, 50), cv::FONT_HERSHEY_COMPLEX, 1, mCsqr);

	std::cout << "droppoint x: " << mXdropPoint << "y: " << mYdropPoint << std::endl;
	mDropPos = {{{mXdropPoint}}, {{mYdropPoint}}};
	auto length = MatrixCalculations::getDistance(mBasePos, mDropPos);
	std::cout << "lengte tussen base en drop : " << length << std::endl;

	while (true)
	{
		int c = cv::waitKey(30);
		cv::imshow("TEST", mInitializeImage);
		if (c == 27) //esc
		{
			break;
		}
	}
}

void MainApplication::run()
{
	bool bId = false, findObject = false;
	std::string lColor_s("empty"), lShape_s("empty"), lId("empty");
	std::vector<std::pair<cv::Point2f, int16_t>> lFound;
	camRecord();

	while (true)
	{
		int c = cv::waitKey(30);
		cv::imshow("TEST", mImage);

		if (c == 32) //spatie
		{
			if (bId)
			{
				std::cout << "[ID]: ";
				std::cin >> lId;

				for (const auto &f : lFound)
				{
					if (f.second == stoi(lId))
					{
						mGoal.x = f.first.x / mPixelsPerCm;
						mGoal.y = f.first.y / mPixelsPerCm;
						Matrix<double, 2, 1>
							lgoalPos = {{{mGoal.x}}, {{mGoal.y}}};

						mBase2GoalAngle = MatrixCalculations::calculateAngle(mBasePos, lgoalPos);

						mBase2goal = MatrixCalculations::getDistance(mBasePos, lgoalPos);
						std::cout
							<< "positie " << lId << " = "
							<< "X: " << mGoal.x << "Y: " << mGoal.y << "baseAngle:" << mBase2GoalAngle << "lengte tussen base en goal:" << mBase2goal << std::endl;

						mBase2drop = MatrixCalculations::getDistance(mBasePos, mDropPos);
						std::cout
							<< "lengte tussen base en drop:" << mBase2drop << std::endl;
						mShapeAngle = P.mShapeAngle;
						bId = false;
						findObject = true;
					}
					else
					{
						bId = true;
						findObject = true;
					}
				}
			}

			input = false;
			if (!findObject)
			{
				std::cout << "[shape][whitespace][color]: ";

				std::cin >> lShape_s >> lColor_s;
				if (P.checkInput(lShape_s, lColor_s))
				{
					std::cout << "Shape: " << lShape_s << " | Color: " << lColor_s << std::endl;

					input = true;
				}
				else
					std::cout << "Shape or color not found, try again please" << std::endl;
			}
		}

		if (input)
		{
			P.setStartValues(lColor_s); //calibration values
			mImage = P.run_program(lColor_s, lShape_s, mImage, coutValue);
			lFound = P.mFoundObjects;
			input = false;
			bId = true;
		}
		if (c == 27) //esc
			break;
	}
}

const std::pair<double, double> MainApplication::getXYCube()
{
	return std::make_pair(mXCube, mYCube);
}

const std::pair<double, double> MainApplication::getXYDropZone()
{
	return std::make_pair(mXdropPoint, mYdropPoint);
}

const cv::Point2f MainApplication::getGoal()
{
	return mGoal;
}

const double MainApplication::getBase2Goal()
{
	return mBase2goal;
}

const double MainApplication::getBase2Drop()
{
	return mBase2drop;
}

const double MainApplication::getBase2GoalAngle()
{
	return mBase2GoalAngle;
}
const double MainApplication::getShapeAngle()
{
	return mShapeAngle;
}
const double MainApplication::getWidth()
{
	return mWidth;
}

// Main function for the program
