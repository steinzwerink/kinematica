#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <al5d_pckg/communicatorAction.h>
#include "al5d_pckg/client.h"

client::client::client() : iterator(0),
                           demo(false),
                           size(3),
                           emergencyStop(false),
                           servos({1, 2, 3}),
                           angles({90, 90, 90}),
                           duration(3000)
{
}
client::client::~client()
{
}

std::vector<uint64_t> client::client::parseMessage(std::string message)
{
  std::vector<uint64_t> messages;
  std::string value;
  for (char &c : message)
  {
    if (c != ' ')
    {
      value += c;
    }
    if (c == ' ')
    {
      messages.push_back((uint64_t)std::stoi(value));
      value = "";
    }
  }
  messages.push_back((uint64_t)std::stoi(value));

  return messages;
}
