#include "my_box.h"
#include "config.h"

#include <RBDyn/parsers/urdf.h>

namespace mc_robots
{

MyBoxRobotModule::MyBoxRobotModule()
: mc_rbdyn::RobotModule(MY_BOX_DESCRIPTION_PATH, "my_box")
{
  init(rbd::parsers::from_urdf_file(urdf_path, true));

  _minimalSelfCollisions.clear();
  _commonSelfCollisions.clear();
}

} // namespace mc_robots

extern "C"
{

ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
{
  names = {"my_box"};
}

ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & name)
{
  ROBOT_MODULE_CHECK_VERSION("my_box")

  if(name == "my_box")
  {
    return new mc_robots::MyBoxRobotModule();
  }

  return nullptr;
}

ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
{
  delete ptr;
}

}