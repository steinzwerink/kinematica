#include "al5d_pckg/Shape.hpp"
#include <map>
//Shape

Shape::Shape(int min_corner, int max_corner, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input_image)
    : Filter(filter_type), min_corner(min_corner), max_corner(max_corner), mX(0), mY(0), ar(ar), erode_size(erode_size), dilate_size(dilate_size), app(app), input_image(input_image)
{
}
Shape::Shape(int min_corner, int max_corner, double mX, double mY, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input_image)
    : Filter(filter_type), min_corner(min_corner), max_corner(max_corner), mX(mX), mY(mY), ar(ar), erode_size(erode_size), dilate_size(dilate_size), app(app), input_image(input_image)
{
}

const int &Shape::get_min_corner()
{
    return this->min_corner;
}

const int &Shape::get_max_corner()
{
    return this->max_corner;
}

const float &Shape::get_app()
{
    return this->app;
}

Shape::~Shape()
{
}

//Square

Square::Square(int min_corner, int max_corner, double mX, double mY, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input_image)
    : Shape(min_corner, max_corner, mX, mY, ar, erode_size, dilate_size, app, filter_type, input_image)
{
}

cv::Mat Square::pipeline()
{

    cv::Mat adaptitvethreshhold, dilated, eroded;

    cv::adaptiveThreshold(input_image, adaptitvethreshhold, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, 2);

    cv::Mat im_th;
    cv::threshold(adaptitvethreshhold, im_th, 220, 255, cv::THRESH_BINARY_INV);

    cv::Mat im_floodfill = im_th.clone();
    cv::floodFill(im_floodfill, cv::Point(0, 0), cv::Scalar(255));
    cv::Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);
    cv::Mat im_out = (im_th | im_floodfill_inv);

    cv::Mat dilate_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_size + 1, 2 * dilate_size + 1), cv::Point(dilate_size, dilate_size));
    cv::Mat erode_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erode_size + 1, 2 * erode_size + 1), cv::Point(erode_size, erode_size));

    cv::erode(im_out, eroded, erode_element);
    cv::dilate(eroded, dilated, dilate_element);

    return dilated;
}

Square::~Square()
{
}

//Rectangle

Rectangle::Rectangle(int min_corner, int max_corner, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input_image)
    : Shape(min_corner, max_corner, ar, erode_size, dilate_size, app, filter_type, input_image)
{
}

cv::Mat Rectangle::pipeline()
{

    cv::Mat adaptitvethreshhold, dilated, eroded;

    cv::adaptiveThreshold(input_image, adaptitvethreshhold, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, 2);

    cv::Mat im_th;
    cv::threshold(adaptitvethreshhold, im_th, 220, 255, cv::THRESH_BINARY_INV);

    cv::Mat im_floodfill = im_th.clone();
    cv::floodFill(im_floodfill, cv::Point(0, 0), cv::Scalar(255));
    cv::Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);
    cv::Mat im_out = (im_th | im_floodfill_inv);

    cv::Mat dilate_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_size + 1, 2 * dilate_size + 1), cv::Point(dilate_size, dilate_size));
    cv::Mat erode_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erode_size + 1, 2 * erode_size + 1), cv::Point(erode_size, erode_size));

    cv::erode(im_out, eroded, erode_element);
    cv::dilate(eroded, dilated, dilate_element);

    return dilated;
}

Rectangle::~Rectangle()
{
}

//Circle

Circle::Circle(int min_corner, int max_corner, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input_image)
    : Shape(min_corner, max_corner, ar, erode_size, dilate_size, app, filter_type, input_image)
{
}
cv::Mat Circle::pipeline()
{
    cv::Mat adaptitvethreshhold, dilated, eroded;

    cv::adaptiveThreshold(input_image, adaptitvethreshhold, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, 2);

    cv::Mat im_th;
    cv::threshold(adaptitvethreshhold, im_th, 220, 255, cv::THRESH_BINARY_INV);

    cv::Mat im_floodfill = im_th.clone();
    cv::floodFill(im_floodfill, cv::Point(0, 0), cv::Scalar(255));
    cv::Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);
    cv::Mat im_out = (im_th | im_floodfill_inv);

    cv::Mat dilate_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_size + 1, 2 * dilate_size + 1), cv::Point(dilate_size, dilate_size));
    cv::Mat erode_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erode_size + 1, 2 * erode_size + 1), cv::Point(erode_size, erode_size));

    cv::erode(im_out, eroded, erode_element);
    cv::dilate(eroded, dilated, dilate_element);

    return dilated;
}

Circle::~Circle()
{
}

//HalfCircle

HalfCircle::HalfCircle(int min_corner, int max_corner, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input_image)
    : Shape(min_corner, max_corner, ar, erode_size, dilate_size, app, filter_type, input_image)
{
}
cv::Mat HalfCircle::pipeline()
{
    cv::Mat adaptitvethreshhold, dilated, eroded;

    cv::adaptiveThreshold(input_image, adaptitvethreshhold, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, 2);

    cv::Mat im_th;
    cv::threshold(adaptitvethreshhold, im_th, 220, 255, cv::THRESH_BINARY_INV);

    cv::Mat im_floodfill = im_th.clone();
    cv::floodFill(im_floodfill, cv::Point(0, 0), cv::Scalar(255));
    cv::Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);
    cv::Mat im_out = (im_th | im_floodfill_inv);

    cv::Mat dilate_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_size + 1, 2 * dilate_size + 1), cv::Point(dilate_size, dilate_size));
    cv::Mat erode_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erode_size + 1, 2 * erode_size + 1), cv::Point(erode_size, erode_size));

    cv::erode(im_out, eroded, erode_element);
    cv::dilate(eroded, dilated, dilate_element);

    return dilated;
}

HalfCircle::~HalfCircle()
{
}

//Triangle

Triangle::Triangle(int min_corner, int max_corner, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input_image)
    : Shape(min_corner, max_corner, ar, erode_size, dilate_size, app, filter_type, input_image)
{
}
cv::Mat Triangle::pipeline()
{
    cv::Mat adaptitvethreshhold, dilated, eroded;

    cv::adaptiveThreshold(input_image, adaptitvethreshhold, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 11, 2);

    cv::Mat im_th;
    cv::threshold(adaptitvethreshhold, im_th, 220, 255, cv::THRESH_BINARY_INV);

    cv::Mat im_floodfill = im_th.clone();
    cv::floodFill(im_floodfill, cv::Point(0, 0), cv::Scalar(255));
    cv::Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);
    cv::Mat im_out = (im_th | im_floodfill_inv);

    cv::Mat dilate_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilate_size + 1, 2 * dilate_size + 1), cv::Point(dilate_size, dilate_size));
    cv::Mat erode_element = getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erode_size + 1, 2 * erode_size + 1), cv::Point(erode_size, erode_size));

    cv::erode(im_out, eroded, erode_element);
    cv::dilate(eroded, dilated, dilate_element);

    return dilated;
}

Triangle::~Triangle()
{
}

const std::vector<std::vector<cv::Point>> Shape::calcontours(const cv::Mat &filtered_image)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(filtered_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    return contours;
}

pairCoordinates Shape::drawimage(const cv::Mat &filtered_image, const cv::Mat original_image, std::string d, float current_app, const int min_corner, const int max_corner, const int coutValues)
{
    //  float lPixelsPerCm;
    static std::map<std::string, enum_shapes> mapShapes;
    mapShapes["calibrate"] = CALIBRATE;
    mapShapes["square"] = SQUARE;
    mapShapes["rectangle"] = RECTANGLE;
    mapShapes["triangle"] = TRIANGLE;
    mapShapes["half_circle"] = HALF_CIRCLE;
    mapShapes["circle"] = CIRCLE;
    cv::Point2f lRectangleVertices[4];
    std::vector<std::vector<cv::Point>> contours = calcontours(filtered_image);
    std::vector<cv::Moments> mu(contours.size());
    cv::Scalar csqr = cv::Scalar(255, 255, 255);

    std::vector<cv::Point2f> mc(contours.size());
    for (uint16_t i = 0; i < contours.size(); i++)
    {
        mu[i] = moments(contours[i], false);
        float x = (float)(mu[i].m10 / mu[i].m00);
        float y = (float)(mu[i].m01 / mu[i].m00);
        mc[i] = cv::Point2f(x, y);
    }

    cv::Mat output = original_image;
    std::vector<std::pair<double, double>> forms;
    std::vector<std::pair<cv::Point2f, int16_t>> foundObjects;
    std::vector<std::vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    std::vector<cv::Point2f> center(contours.size());
    std::vector<float> radius(contours.size());

    for (uint16_t i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], current_app, true);
        boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
        cv::minEnclosingCircle((cv::Mat)contours_poly[i], center[i], radius[i]);
        int16_t x = (int16_t)contours_poly[i].size();
        float ar = ((float)boundRect[i].width / (float)boundRect[i].height);
        auto opp = cv::contourArea(contours_poly[i], false);
        switch (mapShapes[d])
        {

        case CALIBRATE:
        {
            if (x >= min_corner && x <= max_corner && (ar >= 0.90 && ar <= 1.2) && (opp >= 1000 && opp <= 2100))
            {
                std::pair<double, double> lFormId;
                lFormId = std::make_pair(i, 1);
                forms.push_back(lFormId);

                mu[i] = moments(contours[i], false);
                auto lRotatedRect = cv::minAreaRect(contours.at(i));

                float lMaxDistance = 0;

                lRotatedRect.points(lRectangleVertices);

                for (int i = 0; i < 4; i++)
                {
                    double lDistance = (double)cv::norm(
                        lRectangleVertices[i] - lRectangleVertices[(i + 1) % 4]);
                    if (lDistance > lMaxDistance)
                    {
                        lMaxDistance = static_cast<float>(lDistance);
                    }
                }
                mPixelsPerCm =
                    lMaxDistance / static_cast<float>(2.1);

                this->mX = static_cast<double>(mc[i].x);
                this->mY = static_cast<double>(mc[i].y);
            }

            break;
        }
        case SQUARE:
        {
            if (x >= min_corner && x <= max_corner && (ar >= 0.90 && ar <= 1.2) && (opp >= 1000 && opp <= 2100))
            {
                std::pair<double, double> lFormId;
                lFormId = std::make_pair(i, 2);
                forms.push_back(lFormId);

                this->mX = static_cast<double>(mc[i].x);
                this->mY = static_cast<double>(mc[i].y);
            }
            break;
        }
        case RECTANGLE:
        {

            if (x >= min_corner && x <= max_corner && (ar <= 0.91 || ar >= 1.3) && opp >= 1000)
            {
                //forms.push_back(i);
                auto lRotatedRect = cv::minAreaRect(contours.at(i));
                double lShapeAngle = lRotatedRect.angle;
                if (lRotatedRect.size.width < lRotatedRect.size.height)
                    lShapeAngle = lShapeAngle - 90;

                cv::Point2f lVertices[4];
                lRotatedRect.points(lVertices);
                double lMinWidth = DBL_MAX;
                for (int i = 0; i < 4; i++)
                {
                    double lDistance = (double)cv::norm(lVertices[i] - lVertices[(i + 1) % 4]);
                    if (lDistance < lMinWidth)
                    {
                        lMinWidth = (double)lDistance;
                    }
                }
                std::pair<double, double> lFormId;
                lFormId = std::make_pair(i, 2);
                forms.push_back(lFormId);
                this->mX = static_cast<double>(mc[i].x);
                this->mY = static_cast<double>(mc[i].y);
                this->mShapeAngle = lShapeAngle;
                this->mWidth = lMinWidth;
            }
            break;
        }
        case CIRCLE:
        {
            if (opp >= 1400)
            {
                std::pair<double, double> lFormId;
                lFormId = std::make_pair(i, 2);
                forms.push_back(lFormId);

                this->mX = static_cast<double>(mc[i].x);
                this->mY = static_cast<double>(mc[i].y);
            }
            break;
        }
        case HALF_CIRCLE:
        {
            if (x >= min_corner && x <= max_corner && (opp <= 1000 && opp >= 460))
            {
                //forms.push_back(i);
            }
            break;
        }
        case TRIANGLE:
        {
            if (x == min_corner && (opp >= 500 && opp <= 1400))
            {
                //forms.push_back(i);
            }
            break;
        }
        default:
            break;
        }
    }
    int lIdCounter = 0;
    if (forms.size() > 0)
    {

        for (const auto &f : forms)
        {
            if (f.second != 0)
            {

                std::ostringstream shapeArea;
                // Set Fixed -Point Notation
                shapeArea << std::fixed;
                shapeArea << std::setprecision(0);

                if (f.second == 2)
                {
                    shapeArea << "ID:" << f.second + lIdCounter;

                    std::pair<cv::Point2f, double> lObjectLocation = std::make_pair(mc[f.first], static_cast<double>(f.second + lIdCounter));
                    foundObjects.push_back(lObjectLocation);
                    ++lIdCounter;
                }

                // shapeArea << "(" << mc[f.first].x / mPixelsPerCm << "," << mc[f.first].y / mPixelsPerCm << ")";
                std::string strArea = shapeArea.str();

                cv::putText(output, strArea, mc[f.first], cv::FONT_HERSHEY_COMPLEX, 1, csqr);

                cv::drawContours(output, contours_poly, f.first, csqr, 3, 20, std::vector<cv::Vec4i>(), 0, cv::Point());
            }
        }
    }
    else
    {
        cv::putText(output, "Nothing found", cv::Point(50, 300), cv::FONT_HERSHEY_COMPLEX, 1, csqr);
    }
    auto lX = this->mX;
    auto lY = this->mY;
    auto lShapeAngle = this->mShapeAngle;
    auto lWidth = this->mWidth;
    pairCoordinates lPc;
    lPc.sFoundObjects = foundObjects;
    lPc.sX = lX;
    lPc.sY = lY;
    lPc.sShapeAngle = lShapeAngle;
    lPc.sWidth = lWidth;
    lPc.sPixelsPerCm = mPixelsPerCm;
    lPc.image = output;

    return lPc;
}
