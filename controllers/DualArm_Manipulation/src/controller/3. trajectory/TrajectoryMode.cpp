/*
=========================================================================================
    Architecture 
=========================================================================================
Trajectory
│
├── waypoints [W0, W1, W2, ...]
│
├── current waypoint
│
├── current time
│
├── getPose()
│      ├── quintic()
│      ├── spline()
│      └── linear()
│
└── update()
       ├── check if waypoint reached
       ├── advance to next waypoint
       └── calculate current pose

=========================================================================================
    structure
=========================================================================================
                  Trajectory
       ┌──────────────┴──────────────┐
       │                             │
   Waypoints                    Generator
       │                             │
 W0 → W1 → W2                  Quintic/Spline
       │                             │
       └──────────────┬──────────────┘
                      ↓
                desiredPose
                      ↓
              Impedance Task
=========================================================================================
*/


#include <algorithm>
#include <vector>

#include <Eigen/Geometry>

#include <SpaceVecAlg/MathFunc.h>
#include <SpaceVecAlg/PTransform.h>





enum class TrajectoryMode
{
    Once,
    Loop
};

class Trajectory
{
public:
    Trajectory(
                sva::PTransform<double> start,
                const std::vector<sva::PTransform<double>>& waypoints,
                double duration,
                TrajectoryMode mode,
                double positionTolerance
            );

    virtual ~Trajectory() = default;
    sva::PTransform<double> getPose() const{return pose_;}
    void update(double dt,const sva::PTransform<double>& actualPose);

protected:
    virtual sva::PTransform<double> generatePose(double time) const = 0;

    // Accesso controllato per le classi figlie
    const sva::PTransform<double>& start()  const  { return start_; }
    const sva::PTransform<double>& finish() const  { return finish_; }
    double currentTime()                    const  { return currentTime_; }
    double duration()                       const  { return duration_; }

private:
    bool waypointReached(const sva::PTransform<double>& actualPose) const;
    void advanceWaypoint(const sva::PTransform<double>& actualPose);

private:
    TrajectoryMode mode_;
    double positionTolerance_;

    double duration_;
    double currentTime_ = 0.0;

    std::vector<sva::PTransform<double>> waypoints_;
    size_t currentWaypoint_ = 0;

    sva::PTransform<double> start_;
    sva::PTransform<double> finish_;

    sva::PTransform<double> pose_;

    bool finished_ = false;
};



Trajectory::Trajectory(
                        sva::PTransform<double> start,
                        const std::vector<sva::PTransform<double>>& waypoints,
                        double duration,
                        TrajectoryMode mode,
                        double positionTolerance
                        ):  mode_(mode),
                            positionTolerance_(positionTolerance),
                            duration_(duration),
                            waypoints_(waypoints),
                            start_(start),
                            pose_(start)
{
    if (waypoints_.empty()) {   finished_ = true;   return; }
    finish_ = waypoints_.front();
}

void Trajectory::update(
                        double dt,
                        const sva::PTransform<double>& actualPose)
{
    if (finished_)
        return;

    if (waypointReached(actualPose))
    {
        advanceWaypoint(actualPose);
        if (finished_)
            return;
    }

    currentTime_ += dt;
    pose_ = generatePose(currentTime_);
}

bool Trajectory::waypointReached(
                                const sva::PTransform<double>& actualPose) const
{
    double positionError = (actualPose.translation() -finish_.translation()).norm();
    return positionError < positionTolerance_;
}

void Trajectory::advanceWaypoint(const sva::PTransform<double>& actualPose){
    currentTime_ = 0.0;
    start_ = actualPose;

    if (currentWaypoint_ == waypoints_.size() - 1){
        if (mode_ == TrajectoryMode::Once){
            finished_ = true;
            pose_ = actualPose;
            return;
        }
        currentWaypoint_ = 0;
    }
    else { currentWaypoint_++; }
    finish_ = waypoints_[currentWaypoint_];
}

// ===================================================================================



enum class TrajectoryType
{
    Quintic,
    Spline,
    Linear,
    SCurve
};

class InterpolatedTrajectory : public Trajectory
{
public:
    InterpolatedTrajectory(
                            sva::PTransform<double> start,
                            const std::vector<sva::PTransform<double>>& waypoints,
                            double duration,
                            TrajectoryType type,
                            TrajectoryMode mode,
                            double positionTolerance);

protected:
    sva::PTransform<double> generatePose(double time) const override;

private:
    sva::PTransform<double> quintic(double time) const;
    sva::PTransform<double> spline(double time) const;
    sva::PTransform<double> linear(double time) const;
    sva::PTransform<double> sCurve(double time) const;

private:
    TrajectoryType type_;
};


InterpolatedTrajectory::InterpolatedTrajectory(
                                                sva::PTransform<double> start,
                                                const std::vector<sva::PTransform<double>>& waypoints,
                                                double duration,
                                                TrajectoryType type,
                                                TrajectoryMode mode,
                                                double positionTolerance
                                                ):  Trajectory(
                                                                start,
                                                                waypoints,
                                                                duration,
                                                                mode,
                                                                positionTolerance
                                                             ),
                                                    type_(type)
{
}


sva::PTransform<double> InterpolatedTrajectory::quintic(double time) const
{
    double t_norm = std::min(1.0, time / duration());

    if (t_norm >= 1.0)
        return finish();

    const double t2 = t_norm * t_norm;
    const double t3 = t2 * t_norm;

    const double s = t3 * (10.0 + t_norm * (-15.0 + 6.0 * t_norm));

    Eigen::Vector3d startPos = start().translation();
    Eigen::Vector3d targetPos = finish().translation();

    sva::PTransform<double> pose = start();

    pose.translation() = startPos + s * (targetPos - startPos);

    Eigen::Quaterniond q_start(start().rotation());
    Eigen::Quaterniond q_target(finish().rotation());

    pose.rotation() = q_start.slerp(s, q_target).toRotationMatrix();
    
    return pose;
}

sva::PTransform<double> InterpolatedTrajectory::linear(double time) const
{
    // Normalize time to [0, 1]
    const double t = std::clamp(time / duration(), 0.0, 1.0);

    // Start from the initial pose
    sva::PTransform<double> pose = start();

    // -------------------------------------------------------------------------
    // Position: linear interpolation
    // p(t) = p0 + t * (p1 - p0)
    // -------------------------------------------------------------------------
    const Eigen::Vector3d startPos  = start().translation();
    const Eigen::Vector3d targetPos = finish().translation();

    pose.translation() = startPos + t * (targetPos - startPos);

    // -------------------------------------------------------------------------
    // Orientation: quaternion SLERP
    // This gives the shortest smooth rotational interpolation.
    // -------------------------------------------------------------------------
    const Eigen::Quaterniond q_start(start().rotation());
    const Eigen::Quaterniond q_target(finish().rotation());

    pose.rotation() = q_start.slerp(t, q_target).toRotationMatrix();

    return pose;
}

sva::PTransform<double> InterpolatedTrajectory::spline(double time) const
{
    if (duration() <= 0.0)
        return finish();

    // Normalize time to [0, 1]
    const double t = std::clamp(time / duration(), 0.0, 1.0);

    // Cubic smoothstep:
    // s(0) = 0, s(1) = 1
    // s'(0) = 0, s'(1) = 0
    const double t2 = t * t;
    const double t3 = t2 * t;

    const double s = 3.0 * t2 - 2.0 * t3;

    // -------------------------------------------------------------------------
    // Position
    // -------------------------------------------------------------------------
    const Eigen::Vector3d p0 = start().translation();
    const Eigen::Vector3d p1 = finish().translation();

    sva::PTransform<double> pose = start();

    pose.translation() = p0 + s * (p1 - p0);

    // -------------------------------------------------------------------------
    // Orientation
    // -------------------------------------------------------------------------
    const Eigen::Quaterniond q0(start().rotation());
    const Eigen::Quaterniond q1(finish().rotation());

    pose.rotation() = q0.slerp(s, q1).toRotationMatrix();

    return pose;
}


sva::PTransform<double> InterpolatedTrajectory::sCurve(double time) const
{
    if (duration() <= 0.0)
        return finish();

    // Normalize time to [0, 1]
    const double t = std::clamp(time / duration(), 0.0, 1.0);

    // -------------------------------------------------------------------------
    // 7th-order S-curve / smootherstep
    //
    // s(0) = 0
    // s(1) = 1
    //
    // s'(0) = s'(1) = 0
    // s''(0) = s''(1) = 0
    // s'''(0) = s'''(1) = 0
    //
    // -------------------------------------------------------------------------
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double t4 = t3 * t;
    const double t5 = t4 * t;
    const double t6 = t5 * t;
    const double t7 = t6 * t;

    const double s =
          35.0 * t4
        - 84.0 * t5
        + 70.0 * t6
        - 20.0 * t7;

    // -------------------------------------------------------------------------
    // Position
    // -------------------------------------------------------------------------
    const Eigen::Vector3d p0 = start().translation();
    const Eigen::Vector3d p1 = finish().translation();

    sva::PTransform<double> pose = start();

    pose.translation() =
        p0 + s * (p1 - p0);

    // -------------------------------------------------------------------------
    // Orientation
    // -------------------------------------------------------------------------
    const Eigen::Quaterniond q0(start().rotation());
    const Eigen::Quaterniond q1(finish().rotation());

    pose.rotation() =
        q0.slerp(s, q1).toRotationMatrix();

    return pose;
}



sva::PTransform<double>
InterpolatedTrajectory::generatePose(double time) const
{
    switch (type_)
    {
        case TrajectoryType::Quintic:
            return quintic(time);

        case TrajectoryType::Spline:
            return spline(time);

        case TrajectoryType::Linear:
            return linear(time);
        
        case TrajectoryType::SCurve:
            return sCurve(time);
    }
    return finish();
}


// ===================================================================================


class OptimizedTrajectory : public Trajectory
{
public:
    OptimizedTrajectory(
                        sva::PTransform<double> start,
                        const std::vector<sva::PTransform<double>>& waypoints,
                        double duration,
                        TrajectoryMode mode,
                        double positionTolerance);

protected:
    sva::PTransform<double> generatePose(double time) const override;
};

sva::PTransform<double> OptimizedTrajectory::generatePose(double time) const
{
    // TODO: MPC / trajectory optimization
    return finish();
}

OptimizedTrajectory::OptimizedTrajectory(
    sva::PTransform<double> start,
    const std::vector<sva::PTransform<double>>& waypoints,
    double duration,
    TrajectoryMode mode,
    double positionTolerance)
    : Trajectory(
        start,
        waypoints,
        duration,
        mode,
        positionTolerance)
{
}


/*
// ===================================================================================
// Example
// ===================================================================================

// Start pose
sva::PTransform<double> startPose = sva::PTransform<double>::Identity();

// Waypoints
sva::PTransform<double> waypoint1 = sva::PTransform<double>::Identity();
waypoint1.translation() = Eigen::Vector3d(0.5, 0.0, 0.2);

sva::PTransform<double> waypoint2 = sva::PTransform<double>::Identity();
waypoint2.translation() = Eigen::Vector3d(0.8, 0.2, 0.4);


// -----------------------------------------------------------------------------------
// Interpolated trajectory - one waypoint
// -----------------------------------------------------------------------------------

InterpolatedTrajectory interpolatedTrajectory(
    startPose,
    {
        waypoint1
    },
    2.0,
    TrajectoryType::Quintic,
    TrajectoryMode::Once,
    0.01);


// -----------------------------------------------------------------------------------
// Interpolated trajectory - multiple waypoints
// -----------------------------------------------------------------------------------

InterpolatedTrajectory interpolatedTrajectory2(
    startPose,
    {
        waypoint1,
        waypoint2
    },
    2.0,
    TrajectoryType::Quintic,
    TrajectoryMode::Once,
    0.01);


// -----------------------------------------------------------------------------------
// Optimized trajectory
// -----------------------------------------------------------------------------------

OptimizedTrajectory optimizedTrajectory(
    startPose,
    {
        waypoint1,
        waypoint2
    },
    2.0,
    TrajectoryMode::Once,
    0.01);


// -----------------------------------------------------------------------------------
// Update loop
// -----------------------------------------------------------------------------------

// actualPose = current robot pose
//
// double dt = controlPeriod;
//
// interpolatedTrajectory.update(dt, actualPose);
//
// sva::PTransform<double> desiredPose =
//     interpolatedTrajectory.getPose();
//
// Send desiredPose to the controller / impedance task.
*/