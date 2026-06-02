#include "DualArmControl.h"
#include <mc_rbdyn/RobotLoader.h>

DualArmControl::DualArmControl(
    mc_rbdyn::RobotModulePtr rm,
    double dt,
    const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt, config)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  //solver().addTask(postureTask);
  solver().setContacts({{}});

  auto xarm_module = mc_rbdyn::RobotLoader::get_robot_module("xArm7"); 	
  loadRobot(xarm_module, "xarm7_2");
  for(size_t i = 0; i < robots().size(); ++i)
{
  mc_rtc::log::info("After loadRobot: {} -> {}", i, robots().robot(i).name());
}
  robots().robot("xarm7_2").posW(sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0, 0.5, 0)));

  mc_rtc::log::success("DualArmControl init done");

  addCollisions("xarm7", "xarm7_2", {{"*", "*", iDist, sDist, 0}});


}



void DualArmControl::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
 
  std::string EndEffector = "link7";
  LeftEndEffectorTask_ = std::make_shared<mc_tasks::EndEffectorTask>(EndEffector, robots(), 0, 1);
  RightEndEffectorTask_ = std::make_shared<mc_tasks::EndEffectorTask>(EndEffector, robots(), 2, 1);
  
  //leftImpedanceTask_ = std::make_shared<mc_tasks::force::ImpedanceTask>(EndEffector,robots(),0,100.0);
  //rightImpedanceTask_ = std::make_shared<mc_tasks::force::ImpedanceTask>(EndEffector, robots(), 2, 100.0);
  auto & r = robots().robot(0);
  mc_rtc::log::info("============= N SENSORS ================");
  mc_rtc::log::info("Force sensor count = {}", r.forceSensors().size());  
  mc_rtc::log::info("============= END N SENSORS ================");

  mc_rtc::log::info("============= SCAN ================");
  for(const auto & fs : r.forceSensors())
    {
      mc_rtc::log::info("Sensor: {} parent body: {}",fs.name(),fs.parentBody());
    }
  mc_rtc::log::info("============= END SCAN ================");

  
  
  solver().addTask(LeftEndEffectorTask_);
  solver().addTask(RightEndEffectorTask_);

  //solver().addTask(leftImpedanceTask_);
  //solver().addTask(rightImpedanceTask_);


}


void DualArmControl::runOffsetFrame(){

  // End Effector Frame
  std::string EndEffector = "link7";

  auto & leftRobot  = robots().robot(0);
  auto & rightRobot = robots().robot(2);

  sva::PTransformd offset(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.5, 0.0, 0.0));
leftOffsetFrame_  = leftRobot.frame(EndEffector).makeFrame("left_arm_offset_frame", offset);
  rightOffsetFrame_ = rightRobot.frame(EndEffector).makeFrame("right_arm_offset_frame", offset);
  LeftEndEffectorTask_ = std::make_shared<mc_tasks::EndEffectorTask>(*leftOffsetFrame_, 1.0);  // 1.0 è il peso della task
  RightEndEffectorTask_ = std::make_shared<mc_tasks::EndEffectorTask>(*rightOffsetFrame_, 1.0);


  solver().addTask(LeftEndEffectorTask_);
  solver().addTask(RightEndEffectorTask_);

  mc_rtc::log::success("Tasks con offset create correttamente!");





}

void DualArmControl::runDual(){
  runOffsetFrame();

}

void DualArmControl::runIndipendent(){
  
}


bool DualArmControl::run()
{

  switch (phase_)
  {
  case IDLE:
      break;

  case DUAL:
      runDual();
      break;
  
  case INDIPENDENT:
      runIndipendent();
      break;

  default:
      mc_rtc::log::error_and_throw(
        "Porcoddio la fase");
  }



  return mc_control::MCController::run();
}



CONTROLLER_CONSTRUCTOR("DualArmControl", DualArmControl)
