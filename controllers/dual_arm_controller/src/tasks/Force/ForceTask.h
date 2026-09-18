#pragma once

#include <mc_tasks/MetaTask.h>

#include <mc_rbdyn/Robots.h>

#include <Eigen/Core>

#include <mc_rtc/void_ptr.h>
#include <Tasks/QPTasks.h>

#include <array>
#include <map>
#include <vector>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_tasks
{

class ForceTask : public MetaTask
{
public:
  ForceTask(const mc_rbdyn::Robots & robots,
            unsigned int robotIndex,
            const Eigen::Vector3d & targetForce,
            double kp,
            double ki);

  void reset() override;

  Eigen::VectorXd eval() const override;
  Eigen::VectorXd speed() const override;

  void dimWeight(const Eigen::VectorXd & weight) override;
  Eigen::VectorXd dimWeight() const override;

  void selectActiveJoints(
      mc_solver::QPSolver & solver,
      const std::vector<std::string> & joints,
      const std::map<std::string, std::vector<std::array<int, 2>>> & dofs = {}) override;

  void selectUnactiveJoints(
      mc_solver::QPSolver & solver,
      const std::vector<std::string> & joints,
      const std::map<std::string, std::vector<std::array<int, 2>>> & dofs = {}) override;

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  const Eigen::Vector3d & targetForce() const noexcept
  {
    return targetForce_;
  }

  void targetForce(const Eigen::Vector3d & force) noexcept
  {
    targetForce_ = force;
  }

  const Eigen::Vector3d & measuredForce() const noexcept
  {
    return measuredForce_;
  }

  const Eigen::Vector3d & commandedForce() const noexcept
  {
    return commandedForce_;
  }

protected:
  void addToSolver(mc_solver::QPSolver & solver) override;
  void removeFromSolver(mc_solver::QPSolver & solver) override;
  void update(mc_solver::QPSolver & solver) override;

private:
  const mc_rbdyn::Robots & robots_;
  unsigned int robotIndex_;

  mc_rtc::void_ptr forceTask_;

  Eigen::Vector3d targetForce_;
  Eigen::Vector3d measuredForce_;
  Eigen::Vector3d commandedForce_;
  Eigen::Vector3d integralError_;

  Eigen::Vector3d dimWeight_;

  double kp_;
  double ki_;

  bool inSolver_ = false;
};

} // namespace mc_tasks