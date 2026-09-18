#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/RobotModuleMacros.h>

namespace mc_robots
{

struct MyBoxRobotModule : public mc_rbdyn::RobotModule
{
  MyBoxRobotModule();
};

} // namespace mc_robots

extern "C"
{

ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names);

ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & name);

ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr);

}