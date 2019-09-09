#include "al5d_pckg/Filter.hpp"

Filter::Filter(const std::string &filter_type)
    : filter_type(filter_type)
{
}


const cv::Mat Filter::get_HSV(const cv::Mat& original_image)
{
    cv::Mat hsv;
    
    cv::cvtColor(original_image, hsv, CV_BGR2HSV);
   
    return hsv;
}

Filter::~Filter()
{
}
