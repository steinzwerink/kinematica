#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <al5d_pckg/communicatorAction.h>
#include "std_msgs/String.h"
#include "al5d_pckg/lld.h"
#include <pthread.h>

#include "al5d_pckg/moveArm.h"

namespace hld
{

class hld
{
  //class State *current;

protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<al5d_pckg::communicatorAction> as_;
  std::string action_name_;
  al5d_pckg::communicatorActionFeedback feedback_;
  al5d_pckg::communicatorActionResult result_;

private:
  class State *current;
  std::string currentStatename;
  std::string pose_name;
  bool received = false;

public:
  ros::NodeHandle mNh;
  ros::NodeHandlePtr mRosNode;
  ros::Subscriber mRosSubCommands;
  lld::lld lld;
  hld(std::string name, std::string port);
  virtual ~hld();
  al5d_pckg::communicatorGoalConstPtr goalptr;

  void CallBack(const al5d_pckg::moveArmConstPtr &aMsg);
  void setCurrent(State *s);
  void setCurrentStatename(std::string s);
  bool detectlowlvl();
  void executeCB(const al5d_pckg::communicatorGoalConstPtr &goal);
  void startListening();
  void stopListening();
  void setAborted();
  void setPreempted();
  void setSucceeded();
  void publishFeedback();
  bool isPreempt();
  void setReceived(const bool inputreceived);
  const bool &getReceived();
  const std::string &getActionName();
  const std::string &getPosName();
  const std::string &getCurrentStatename();
  void setFeedback(int16_t angle, uint16_t time);
  void setResult();
};
} // namespace hld