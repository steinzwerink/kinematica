#include "al5d_pckg/Color.hpp"

//Color

Color::Color(const std::string &filter_type, const cv::Mat &input_image)
    : Filter(filter_type), input_image(input_image)
{
}

Color::~Color()
{
}

//Red

Red::Red(const std::string &filter_type, const cv::Mat &input_image)
    : Color(filter_type, input_image)
{
}

cv::Mat Red::pipeline(int h_low, int s_low, int v_low, int h_upper, int s_upper, int v_upper, int h_low_max, int h_upper_max)
{
   
    cv::Mat hsv_image, lower_range, upper_range, hue_image;
    hsv_image = this->get_HSV(input_image);

    cv::inRange(hsv_image, cv::Scalar(h_low, s_low, v_low), cv::Scalar(h_low_max, 255, 255), lower_range);
    cv::inRange(hsv_image, cv::Scalar(h_upper, s_upper, v_upper), cv::Scalar(h_upper_max, 255, 255), upper_range);
    cv::addWeighted(lower_range, 1.0, upper_range, 1.0, 0.0, hue_image);

    return hue_image;
}

Red::~Red()
{
}

//Green

Green::Green(const std::string &filter_type, const cv::Mat &input_image)
    : Color(filter_type, input_image)
{
}

cv::Mat Green::pipeline(int h_low, int s_low, int v_low, int h_upper, int s_upper, int v_upper, int h_low_max, int h_upper_max)
{
    cv::Mat hsv_image, lower_range, upper_range, hue_image;
    hsv_image = this->get_HSV(input_image);
    cv::inRange(hsv_image, cv::Scalar(h_low, s_low, v_low), cv::Scalar(h_low_max, 255, 255), lower_range);
    cv::inRange(hsv_image, cv::Scalar(h_upper, s_upper, v_upper), cv::Scalar(h_upper_max, 255, 255), upper_range);
    cv::addWeighted(lower_range, 1.0, upper_range, 1.0, 0.0, hue_image);
    return hue_image;
}

Green::~Green()
{
}

//Blue

Blue::Blue(const std::string &filter_type, const cv::Mat &input_image)
    : Color(filter_type, input_image)
{
}

cv::Mat Blue::pipeline(int h_low, int s_low, int v_low, int h_upper, int s_upper, int v_upper, int h_low_max, int h_upper_max)
{
    cv::Mat hsv_image, lower_range, upper_range, hue_image;
    hsv_image = this->get_HSV(input_image);
    cv::inRange(hsv_image, cv::Scalar(h_low, s_low, v_low), cv::Scalar(h_low_max, 255, 255), lower_range);
    cv::inRange(hsv_image, cv::Scalar(h_upper, s_upper, v_upper), cv::Scalar(h_upper_max, 255, 255), upper_range);
    cv::addWeighted(lower_range, 1.0, upper_range, 1.0, 0.0, hue_image);
    return hue_image;
}

Blue::~Blue()
{
}

//Yellow

Yellow::Yellow(const std::string &filter_type, const cv::Mat &input_image)
    : Color(filter_type, input_image)
{
}

cv::Mat Yellow::pipeline(int h_low, int s_low, int v_low, int h_upper, int s_upper, int v_upper, int h_low_max, int h_upper_max)
{
    cv::Mat hsv_image, lower_range, upper_range, hue_image;
    hsv_image = this->get_HSV(input_image);
    cv::inRange(hsv_image, cv::Scalar(h_low, s_low, v_low), cv::Scalar(h_low_max, 255, 255), lower_range);
    cv::inRange(hsv_image, cv::Scalar(h_upper, s_upper, v_upper), cv::Scalar(h_upper_max, 255, 255), upper_range);
    cv::addWeighted(lower_range, 1.0, upper_range, 1.0, 0.0, hue_image);
    return hue_image;
}

Yellow::~Yellow()
{
}

//White

White::White(const std::string &filter_type, const cv::Mat &input_image)
    : Color(filter_type, input_image)
{
}

cv::Mat White::pipeline(int h_low, int s_low, int v_low, int h_upper, int s_upper, int v_upper, int h_low_max, int h_upper_max)
{
    cv::Mat hsv_image, lower_range, upper_range, hue_image;
    hsv_image = this->get_HSV(input_image);
    cv::inRange(hsv_image, cv::Scalar(h_low, s_low, v_low), cv::Scalar(h_low_max, 255, 255), lower_range);
    cv::inRange(hsv_image, cv::Scalar(h_upper, s_upper, v_upper), cv::Scalar(h_upper_max, 255, 255), upper_range);
    cv::addWeighted(lower_range, 1.0, upper_range, 1.0, 0.0, hue_image);
    return hue_image;
}

White::~White()
{
}
