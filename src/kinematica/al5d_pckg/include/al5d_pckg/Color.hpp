/** @file Filter.h
 *  @brief Elke kleur heeft een eigen klasse.
 *
 *  Elke kleur heeft een eigen klasse.
 *
 * 
 */

#ifndef COLOR_H
#define COLOR_H

#include "al5d_pckg/Filter.hpp"
/** @brief enum met alle mogelijke kleuren erin.
 *
 * 
 */
enum enum_colors
{
  RED,
  GREEN,
  BLUE,
  YELLOW,
  WHITE,
  BLACK
};
/** @brief klasse color 
 *
 *  
 *
 *  @param filter_type welke kleur er moet worden gefilterd.
 * @param input_image de foto waar er uit gefilterd moet worden.
 * @return pipeline op basis van een input image en filter_type wordt er een niewe gefilterde foto teruggeven met daarop 1 kleur.
 */
class Color : public Filter
{
protected:
  cv::Mat input_image;

public:
  Color(const std::string &filter_type, const cv::Mat &input_image);
  virtual cv::Mat pipeline(int h_low, int s_low, int v_low, int h_upper, int s_upper, int v_upper, int h_low_max, int h_upper_max) = 0;
  virtual ~Color();
};

class Red : public Color
{
public:
  Red(const std::string &filter_type, const cv::Mat &input);
  cv::Mat pipeline(int h_low, int s_low, int v_low, int h_upper, int s_upper, int v_upper, int h_low_max, int h_upper_max);
  virtual ~Red();
};

class Green : public Color
{
public:
  Green(const std::string &filter_type, const cv::Mat &input);
  cv::Mat pipeline(int h_low, int s_low, int v_low, int h_upper, int s_upper, int v_upper, int h_low_max, int h_upper_max);
  virtual ~Green();
};

class Blue : public Color
{
public:
  Blue(const std::string &filter_type, const cv::Mat &input);
  cv::Mat pipeline(int h_low, int s_low, int v_low, int h_upper, int s_upper, int v_upper, int h_low_max, int h_upper_max);

  virtual ~Blue();
};

class Yellow : public Color
{
public:
  Yellow(const std::string &filter_type, const cv::Mat &input);
  cv::Mat pipeline(int h_low, int s_low, int v_low, int h_upper, int s_upper, int v_upper, int h_low_max, int h_upper_max);

  virtual ~Yellow();
};

class White : public Color
{
public:
  White(const std::string &filter_type, const cv::Mat &input);
  cv::Mat pipeline(int h_low, int s_low, int v_low, int h_upper, int s_upper, int v_upper, int h_low_max, int h_upper_max);

  virtual ~White();
};

#endif