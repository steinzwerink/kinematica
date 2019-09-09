#include "al5d_pckg/lld.h"
#include "al5d_pckg/ConstValues.h"

lld::lld::lld() : mSize(5)
{
}

lld::lld::~lld()
{
}

bool lld::lld::detectDriver()
{
    bool armdetected = true;
    try
    {
        boost::asio::io_service ioservice;
        boost::asio::serial_port serial(ioservice, this->port);
    }
    catch (const std::exception &e)
    {
        armdetected = false;
    }
    return armdetected;
}

void lld::lld::setPort(const std::string port)
{
    this->port = port;
}

unsigned int lld::lld::angle2Pwm(int aAngle, int istart, int istop, int ostart, int ostop)
{
    unsigned int lPwmRange = ostop - ostart;
    ROS_WARN("PWM range: %i", lPwmRange);
    unsigned int lAngleRange = static_cast<unsigned int>(std::abs(istop - istart));
    ROS_WARN("lAngleRange: %i", lAngleRange);
    ROS_WARN("aAngle: istart:istop %i,%i,%i", aAngle, istart, istop);
    unsigned int lRemapped = remap(aAngle, istart, istop, 0, lAngleRange);
    ROS_WARN("lRemapped: %i", lRemapped);
    double lFactor = static_cast<double>(lRemapped) / static_cast<double>(lAngleRange);
    ROS_WARN("lFactor: %i", lFactor);

    int16_t pwm = al5d::cMin + (lPwmRange * lFactor);
    ROS_WARN("pwm: %i", pwm);

    return pwm;
}

unsigned int lld::lld::remap(int value, int istart, int istop, int ostart, int ostop) const
{
    return (ostop - ostart) * (value - istart) / (istop - istart) + ostart;
}

bool lld::lld::checkRange(int16_t angle, uint16_t servo)
{
    bool check;
    int16_t min, max;

    if (servo == 2)
    {
        min = 0;
        max = 135;
        if (angle >= min && angle <= max)
            check = true;
        else
            check = false;
    }
    else if (servo == 1)
    {
        min = -90;
        max = 30;
        if (angle >= min && angle <= max)
            check = true;
        else
            check = false;
    }
    else if (servo == 3)
    {
        min = -90;
        max = 90;
        if (angle >= min && angle <= max)
            check = true;
        else
            check = false;
    }
    else
    {

        check = true;
    }
    return check;
}

void lld::lld::sendAction(const uint16_t pose, const std::vector<uint16_t> servo, const std::vector<int16_t> angle, const uint16_t duration, const uint16_t size)
{
    int16_t min, max;
    std::vector<std::string> strings;
    std::string pose_name, c_sqc, c_time = (" T");
    c_time += std::to_string(duration);
    c_time += ("\r");

    if (pose == 1)
    {
        for (int i = 0; i < size; ++i)
        {
            if (!checkRange(angle[i], servo[i]))
                ROS_WARN("Out of range:\n Servo: %i Angle: %i", servo[i], angle[i]);
            else
            {
                c_sqc += (" #");
                c_sqc += std::to_string(servo[i]);
                c_sqc += (" P");
                min = static_cast<int16_t>(al5d::cServos[(servo[i] * 2)]);
                max = static_cast<int16_t>(al5d::cServos[(servo[i] * 2 + 1)]);

                ROS_WARN("min:%i max:%i \n Servo: %i", min, max, servo[i]);
                int16_t pwm = angle2Pwm(angle[i], min, max, al5d::cMin, al5d::cMax);
                pwm += al5d::cPwmComp[servo[i]];
                c_sqc += std::to_string(pwm);
            }
        }
        c_sqc += c_time;
        strings.push_back(c_sqc);
    }
    else
    {
        for (int i = 0; i <= mSize; ++i)
        {
            c_sqc = ("STOP ");
            c_sqc += std::to_string(i);
            c_sqc += ("\r");
            strings.push_back(c_sqc);
        }
    }

    boost::asio::io_service ioservice;
    boost::asio::serial_port serial(ioservice, this->port);

    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));

    for (const std::string &s : strings)
    {

        boost::asio::streambuf b;
        std::ostream os(&b);
        os << s << "\r";
        boost::asio::write(serial, b.data());
        os.flush();
    }
    if (serial.is_open())
    {
        serial.close();
    }
}
