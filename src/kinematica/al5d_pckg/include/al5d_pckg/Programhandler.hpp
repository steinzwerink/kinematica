/** @brief input vanuit de commandline of batchfile wordt hier afgehandeld.
 *
 * op basis van de opgeven kleuren en vormen worden er de daarbijhorende filters en waarden gekozen via een switchcase.
 */

#ifndef PROGRAMHANDLER_H
#define PROGRAMHANDLER_H

#include "Color.hpp"
#include "Shape.hpp"
#include "al5d_pckg/position.h"
#include <stdint.h>
/** 
 *
 * @param modus verschillende standen waarin het programma zich kan bevinden en hoe de gebruiker input moet leveren.

 */
enum modus
{
  interactive_modus,
  batch_modus,
  default_modus

};

/** @brief input vanuit de commandline of batchfile wordt hier afgehandeld.
 *
 * @param spec houdt de kleur en vorm bij elkaar in een datacontainer.
 * @param batch_file het pad waar de batchfile zich bevind.
 * @param start_clock startwaarde die gebruikt wordt voor het berekenen van de clockticken.
 * @param end_clock eindwaarde die gebruikt wordt voor het berekenen van de clockticken.
 * @param h_low hue waarde voor de inrange filter. Gebruikt voor het filteren van een specifieke kleur.
 * @param s_low saturation waarde voor de inrange filter. Gebruikt voor het filteren van een specifieke kleur.
 * @param v_low value waarde voor de inrange filter. Gebruikt voor het filteren van een specifieke kleur.
 */

class Programhandler
{
private:
  std::tuple<std::string, std::string> spec;
  std::string batch_file;
  modus program_modus;
  std::clock_t start_clock;
  std::clock_t end_clock;
  int h_low, s_low, v_low, h_upper, s_upper, v_upper, h_low_max, h_upper_max;

public:
  Programhandler(std::string batchFile, modus program_modus);
  cv::Mat run_program(std::string s, std::string d, const cv::Mat &origional_image, const int coutValue);
  void sendCoordinates();
  void initialize();
  const std::vector<std::tuple<std::string, std::string>> handlefile();
  bool checkInput(const std::string &shape, const std::string &color);
  void handleInput();
  void startClock();
  std::clock_t getClock();
  void set_program_modus(const modus m);
  const modus &get_program_modus();
  void set_batch_file(const std::string &batch_file_arg);
  void trackbar(char *input);
  void setStartValues(std::string c_type);
  virtual ~Programhandler();

  std::map<std::string, enum_shapes> mapShapes;
  std::map<std::string, enum_colors> mapColors;
  std::pair<double, double> mCoordinates;
  std::vector<std::pair<cv::Point2f, int16_t>> mFoundObjects;
  float mPixelsPerCm;
  double mShapeAngle;
  double mWidth;
};

#endif