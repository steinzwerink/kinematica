#include "hld.h"
#include <iostream>
namespace hld
{
class State
{
public:
  virtual void onDo(hld *m) = 0;

};

class Ready : public State
{
private:
  std::string stateName;

public:
  Ready(hld *m);
  virtual ~Ready();
  void onDo(hld *m);
  
};

class Initialized : public State
{
private:
public:
  Initialized(hld *m);
  virtual ~Initialized();
  void onDo(hld *m);
 
};

class Active : public State
{
private:
public:
  Active(hld *m);
  virtual ~Active();
  void onDo(hld *m);
 
};

class Emergency : public State
{
private:
  std::string stateName;

public:
  Emergency(hld *m);
  virtual ~Emergency();
  void onDo(hld *m);
 
};

} // namespace hld