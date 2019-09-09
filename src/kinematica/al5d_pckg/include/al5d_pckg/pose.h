#include <iostream>
#include <vector>
#include <string>
#include <thread>

namespace pose
{
class pose
{
  private:
    
  public:
    std::string pose_name;
    std::vector<uint16_t> servo;
    std::vector<int16_t> angle;
    uint16_t duration; 
    pose();
    pose(std::string pose_name, std::vector<uint16_t> servo,std::vector<int16_t> angle, uint16_t duration);
    virtual ~pose();
   
};

} // namespace client


