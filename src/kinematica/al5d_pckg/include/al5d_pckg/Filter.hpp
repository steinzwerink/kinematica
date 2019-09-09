#ifndef FILTER_H
#define FILTER_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <map>
#include <string>
#include <fstream>
#include <vector>
#include <stdint.h>
#include <iomanip>

/** @brief filter klasse waar de klasse color en shape van overerven.
 *
 * Heeft een virtuele functie pipeline die door elke overgeerfde klasse kan worden gebruikt om zelf in te vullen.
 *@param filter_type wat gefilterd moet worden, bijvoorbeeld rectangle. 
 */

class Filter
{
protected:
  std::string filter_type;

public:
  Filter(const std::string &filter_type);
  const std::string &getfilter_type();
  const cv::Mat get_HSV(const cv::Mat &original_image);

  virtual ~Filter();
};

#endif