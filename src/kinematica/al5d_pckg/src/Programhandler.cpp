#include "al5d_pckg/Programhandler.hpp"

Programhandler::Programhandler(std::string batchFile, modus program_modus)
    : batch_file(batchFile), program_modus(program_modus)
{
    mapShapes["calibrate"] = CALIBRATE;
    mapShapes["square"] = SQUARE;
    mapShapes["rectangle"] = RECTANGLE;
    mapShapes["triangle"] = TRIANGLE;
    mapShapes["half_circle"] = HALF_CIRCLE;
    mapShapes["circle"] = CIRCLE;
    mapColors["red"] = RED;
    mapColors["green"] = GREEN;
    mapColors["blue"] = BLUE;
    mapColors["yellow"] = YELLOW;
    mapColors["white"] = WHITE;
    mapColors["black"] = BLACK;
}

cv::Mat Programhandler::run_program(std::string s, std::string d, const cv::Mat &originial_image, const int coutValue)
{

    cv::Mat color_filter;
    cv::Mat form_filter;
    Color *color;
    Shape *shape;

    switch (mapColors[s])
    {
    case RED:
    {

        Red red("Red", originial_image);
        color = &red;
        color_filter = color->pipeline(h_low, s_low, v_low, h_upper, s_upper, v_upper, h_low_max, h_upper_max);
        break;
    }
    case GREEN:
    {
        // std::cout << s << std::endl;
        Green green("Green", originial_image);
        color = &green;
        color_filter = color->pipeline(h_low, s_low, v_low, h_upper, s_upper, v_upper, h_low_max, h_upper_max);
        break;
    }
    case BLUE:
    {
        Blue blue("Blue", originial_image);
        color = &blue;
        color_filter = color->pipeline(h_low, s_low, v_low, h_upper, s_upper, v_upper, h_low_max, h_upper_max);
    }
    break;
    case YELLOW:
    {
        Yellow yellow("Yellow", originial_image);
        color = &yellow;
        color_filter = color->pipeline(h_low, s_low, v_low, h_upper, s_upper, v_upper, h_low_max, h_upper_max);
    }
    break;
    case WHITE:
    {
        White white("White", originial_image);
        color = &white;
        color_filter = color->pipeline(h_low, s_low, v_low, h_upper, s_upper, v_upper, h_low_max, h_upper_max);
    }
    break;

    default:
        break;
    }

    switch (mapShapes[d])
    {
    case CALIBRATE:
    {
        Square square(4, 5, 0, 0, 2, 5, 1, (float)6.5, "Square", color_filter); //let op pas de erode size  aan naar 15 wss
        shape = &square;
        form_filter = square.pipeline();
        break;
    }
    case SQUARE:
    {
        Square square(4, 5, 0, 0, 2, 5, 1, (float)6.5, "Square", color_filter); //let op pas de erode size  aan naar 15 wss
        shape = &square;
        form_filter = square.pipeline();
        break;
    }
    case RECTANGLE:
    {
        Rectangle rec(4, 7, 2, 5, 1, (float)6.5, "Rectangle", color_filter);
        shape = &rec;
        form_filter = rec.pipeline();
        break;
    }
    case CIRCLE:
    {
        Circle circle(7, 8, 2, 3, 1, (float)5.3, "Circle", color_filter);
        shape = &circle;
        form_filter = circle.pipeline();
        break;
    }
    case HALF_CIRCLE:
    {
        HalfCircle half_circle(6, 8, 2, 3, 1, (float)3.5, "Half_circle", color_filter);
        shape = &half_circle;
        form_filter = half_circle.pipeline();
        break;
    }
    case TRIANGLE:
    {
        Triangle triangle(3, 4, 2, 5, 1, (float)6.5, "triangle", color_filter);
        shape = &triangle;
        form_filter = triangle.pipeline();
        break;
    }
    default:
        break;
    }

    if (coutValue == 2)
    {
        return color_filter;
    }
    else
    {

        auto i = shape->drawimage(form_filter, originial_image, d, shape->get_app(), shape->get_min_corner(), shape->get_max_corner(), coutValue);
        mCoordinates = std::make_pair(i.sX, i.sY);
        mFoundObjects = i.sFoundObjects;
        mShapeAngle = i.sShapeAngle;
        mPixelsPerCm = i.sPixelsPerCm;
        return i.image;
    }
}

void Programhandler::sendCoordinates()
{
}

const std::vector<std::tuple<std::string, std::string>> Programhandler::handlefile()
{
    std::string line, shape, color;
    std::ifstream myfile(batch_file);
    std::vector<std::tuple<std::string, std::string>> v;
    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            std::stringstream s(line);
            s >> shape >> color;
            if (!(shape[0] == '#' || color[0] == '#'))
            {

                spec = std::make_tuple(shape, color);
                v.push_back(spec);
            }
        }
        myfile.close();
    }
    return v;
}

bool Programhandler::checkInput(const std::string &shape_s, const std::string &color_s)
{
    bool foundShape = false;
    bool foundColor = false;

    for (const auto &sh : mapShapes)
    {
        if (sh.first == shape_s)
        {
            // std::cout << sh.first << std::endl;
            foundShape = true;
        }
    }
    for (const auto &co : mapColors)
    {
        if (co.first == color_s)
        {
            foundColor = true;
        }
    }
    return foundShape && foundColor;
}

void Programhandler::set_program_modus(const modus m)
{
    this->program_modus = m;
}
const modus &Programhandler::get_program_modus()
{

    return this->program_modus;
}
void Programhandler::startClock()
{

    this->start_clock = std::clock();
}

std::clock_t Programhandler::getClock()
{

    this->end_clock = std::clock();
    return (this->end_clock - this->start_clock);
}
void Programhandler::set_batch_file(const std::string &batch_file_arg)
{
    this->batch_file = batch_file_arg;
}

void Programhandler::trackbar(char *input)
{
    std::string c_type;
    std::stringstream s(input);
    s >> c_type;
    setStartValues(c_type);
    cv::createTrackbar("Hue_low", "TEST", &h_low, 255, 0);
    cv::createTrackbar("Saturation_low", "TEST", &s_low, 255, 0);
    cv::createTrackbar("Value_low", "TEST", &v_low, 255, 0);
    cv::createTrackbar("Hue_low_max", "TEST", &h_low_max, 255, 0);
    cv::createTrackbar("Hue_upper", "TEST", &h_upper, 255, 0);
    cv::createTrackbar("Saturation_upper", "TEST", &s_upper, 255, 0);
    cv::createTrackbar("Value_upper", "TEST", &v_upper, 255, 0);
    cv::createTrackbar("Hue_upper_max", "TEST", &h_upper_max, 255, 0);
}
void Programhandler::setStartValues(std::string c_type)
{
    if (c_type == "red")
    {
        //  std::cout << " succes" << std::endl;
        h_low = 0;
        s_low = 93;
        v_low = 55;
        h_low_max = 4;
        h_upper = 160;
        s_upper = 108;
        v_upper = 10;
        h_upper_max = 185;
    }
    else if (c_type == "yellow")
    {
        h_low = 23;
        s_low = 112;
        v_low = 59;
        h_low_max = 94;
        h_upper = 115;
        s_upper = 157;
        v_upper = 0;
        h_upper_max = 0;
    }
    else if (c_type == "green")
    {
        h_low = 89;
        s_low = 134;
        v_low = 21;
        h_low_max = 105;
        h_upper = 186;
        s_upper = 0;
        v_upper = 0;
        h_upper_max = 0;
    }
    else if (c_type == "blue")
    {
        h_low = 113;
        s_low = 73;
        v_low = 59;
        h_low_max = 132;
        h_upper = 255;
        s_upper = 104;
        v_upper = 0;
        h_upper_max = 107;
    }
    else if (c_type == "white")
    {
        h_low = 0;
        s_low = 61;
        v_low = 255;
        h_low_max = 0;
        h_upper = 102;
        s_upper = 45;
        v_upper = 241;
        h_upper_max = 120;
    }
}
Programhandler::~Programhandler()
{
}