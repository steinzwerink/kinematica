/** @file Shape.h
 *  @brief Elke vorm heeft een eigen klasse.
 *
 *  Elke vorm heeft een eigen klasse, want voor elke vorm moeten de filters anders worden gebruikt.
 *
 * 
 */
#ifndef SHAPE_H
#define SHAPE_H
#include <utility>
#include <functional>
#include "al5d_pckg/Filter.hpp"
/** @brief enum met alle mogelijke vormen erin.
 *
 * 
 */
enum enum_shapes
{
  CALIBRATE,
  SQUARE,
  RECTANGLE,
  TRIANGLE,
  HALF_CIRCLE,
  CIRCLE
};
typedef struct
{
  double sX;
  double sY;
  double sShapeAngle;
  double sWidth;
  std::vector<std::pair<cv::Point2f, int16_t>> sFoundObjects;
  float sPixelsPerCm;
  cv::Mat image;
} pairCoordinates;
/** @brief klasse Shape 
 *
 *  @param min_corner de minimale hoeveelheid hoeken waaraan voldaan moet worden.
 * @param max_corner de maximale hoeveelheid hoeken waaraan voldaan moet worden.
 * @param ar de ratio tussen de lengte en de breedte van een object, wordt gebruikt om het verschil tussen een vierkant en rechthoek te bepalen.
 * @param erode_size de kernelsize die opgegeven moet worden voor het erode filter.
 * @param dilate_size de kernelsize die opgegeven moet worden voor het dilate filter.
 * @param app inout waarde voor de approxpoly filter.
 * @param input_image de foto waar er uit gefilterd moet worden.
 * @param coutValues waarde die bijhoudt of de waardes op het scherm moeten worden geprint of naar de cout moeten.
 * 
 */
class Shape : public Filter
{
protected:
public:
  int min_corner;
  int max_corner;
  double mX;
  double mY;
  int ar;
  int erode_size;
  int dilate_size;
  float app;
  cv::Mat input_image;
  int coutValues;
  float mPixelsPerCm;
  double mShapeAngle;
  double mWidth;
  double mDropZoneX;
  double mDropZoneY;
  Shape(int min_corner, int max_corner, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input);
  Shape(int min_corner, int max_corner, double mX, double mY, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input);

  const std::vector<int> &get_amount_of_corners();
  const int &get_ar();
  void setCout(int coutValues);
  const int &getCout();
  const int &get_erode_size();
  const int &get_dilate_size();
  const float &get_app();
  const int &get_min_corner();
  const int &get_max_corner();

  /** @brief berekend de contouren van een object.
 *
 *  @param filtered_image de gefilterde foto waar contouren ingevonden kunnen worden.
 *  @return geeft een vector van vectoren terug met points daarin van de gevonden contouren.
 */
  const std::vector<std::vector<cv::Point>> calcontours(const cv::Mat &filtered_image);

  /** @brief tekend de contouren,oppervlakte en x en y van een gevonden object.
 *
 * @param filtered_image de gefilterde foto waar contouren ingevonden kunnen worden.
 * @param original_image foto waarop getekend wordt.
 * @param d string met de door de gebruiker opgegeven vorm die getekend moet worden.
 * @param min_corner de minimale hoeveelheid hoeken waaraan voldaan moet worden.
 * @param max_corner de maximale hoeveelheid hoeken waaraan voldaan moet worden.
 * @param ar de ratio tussen de lengte en de breedte van een object, wordt gebruikt om het verschil tussen een vierkant en rechthoek te bepalen.
 * @param app input waarde voor de approxpoly filter.
 * @param input_image de foto waar er uit gefilterd moet worden.
 * 
 *  @return geeft een foto terug in de vorm van een cv::Mat.
 */
  pairCoordinates drawimage(const cv::Mat &filtered_image, const cv::Mat original_image, std::string d, float current_app, const int min_corner, const int max_corner, const int coutValues);
  cv::Mat pipeline();
  virtual ~Shape();
};

class Rectangle : public Shape
{
public:
  Rectangle(int min_corner, int max_corner, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input);
  cv::Mat pipeline();
  virtual ~Rectangle();
};

class Square : public Shape
{
public:
  Square(int min_corner, int max_corner, double mX, double mY, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input);
  cv::Mat pipeline();
  virtual ~Square();
  double mX;
  double mY;
};

class Circle : public Shape
{
public:
  Circle(int min_corner, int max_corner, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input);
  cv::Mat pipeline();
  virtual ~Circle();
};

class HalfCircle : public Shape
{
public:
  HalfCircle(int min_corner, int max_corner, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input);
  cv::Mat pipeline();
  virtual ~HalfCircle();
};

class Triangle : public Shape
{
public:
  Triangle(int min_corner, int max_corner, const int ar, const int erode_size, const int dilate_size, const float app, const std::string &filter_type, const cv::Mat &input);
  cv::Mat pipeline();
  virtual ~Triangle();
};

#endif