#include <ros/ros.h>
#include <string>
#include "al5d_pckg/pose.h"

pose::pose::pose(){
}

pose::pose::pose(std::string pose_name, std::vector<uint16_t> servo,std::vector<int16_t> angle, uint16_t duration)
: pose_name(pose_name), servo(servo), angle(angle), duration(duration) 
{
}

pose::pose::~pose()
{
}