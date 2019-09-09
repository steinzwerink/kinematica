#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include "pose.h"
namespace client
{
class client
{
public:
  client();
  virtual ~client();
  std::vector<uint64_t> parseMessage(std::string message);

  int iterator;
  bool demo;
  uint16_t size;
  bool emergencyStop;
  std::vector<uint16_t> servos;
  std::vector<int16_t> angles;
  uint16_t duration;
};
} // namespace client