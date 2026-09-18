/*
=========================================================================================
    Architecture 
=========================================================================================
Trajectory
│
├── waypoints
│    ├── W0
│    ├── W1
│    ├── W2
│    └── ...
│
├── current waypoint
│
├── current time
│
├── getPose()
│      │
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
                      │
       ┌──────────────┴──────────────┐
       │                             │
   Waypoints                    Generator
       │                             │
 W0 → W1 → W2                  Quintic/Spline
       │                             │
       └──────────────┬──────────────┘
                      ↓
                desiredPose
                      │
                      ↓
              Impedance Task
=========================================================================================
*/


#include <algorithm>
#include <vector>
#include <Eigen/Geometry>

/*
enum class TrajectoryType
{
    Quintic,
    Spline,
    Linear
};

enum class TrajectoryMode
{
    Once,
    Loop
};



class Trajectory{
    public:
        Trajectory(
            sva::PTransformd start,
            const std::vector<sva::PTransformd>& waypoints,
            double duration,
            TrajectoryType type,
            TrajectoryMode mode,
            double positionTolerance);

        sva::PTransformd getPose() const { return pose_; }
        void update(double dt,const sva::PTransformd& actualPose);

    private:
        // Configuration
            TrajectoryType type_;
            TrajectoryMode mode_;
            double positionTolerance_;
            double totalTime_= 0.0;
            double currentTime_ = 0.0;
        // Waypoints
            std::vector<sva::PTransformd> waypoints_;    
            size_t currentWaypoint_ = 0;
        // Current trajectory segment
            sva::PTransformd start_;
            sva::PTransformd finish_;
            bool finished_ = false;
        // Current desired pose
            sva::PTransformd pose_ = {Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero()}; 

        sva::PTransformd generatePose(double time) const;

    private:
        bool waypointReached(const sva::PTransformd& actualPose) const;
        void advanceWaypoint(const sva::PTransformd& actualPose);
        // Interpolation methods
        sva::PTransformd quintic(double time);
        sva::PTransformd spline(double time);
        sva::PTransformd linear(double time);

};



Trajectory::Trajectory(
    sva::PTransformd start,
    const std::vector<sva::PTransformd>& waypoints,
    double duration,
    TrajectoryType type,
    TrajectoryMode mode,
    double positionTolerance)
    : type_(type),
      mode_(mode),
      positionTolerance_(positionTolerance),
      totalTime_(duration),
      waypoints_(waypoints),
      start_(start),
      pose_(start){
    if (waypoints_.empty()){finished_ = true; return;}
    finish_ = waypoints_.front();
}


bool Trajectory::waypointReached(const sva::PTransformd& actualPose) const{
    double positionError = (actualPose.translation() -finish_.translation()).norm();
    return positionError < positionTolerance_;
}





void Trajectory::update(double dt,const sva::PTransformd& actualPose){
    if (finished_){return;}
    currentTime_ += dt;
    if  (waypointReached(actualPose))
        {advanceWaypoint(actualPose);} 
    if (finished_) {return;}
    pose_ = generatePose(currentTime_);
}

sva::PTransformd Trajectory::generatePose(double time) const{
    // Generate desired pose
    switch (type_){
        case TrajectoryType::Quintic:
            return quintic(time);  break;
        case TrajectoryType::Spline:
            return spline(time);   break;
        case TrajectoryType::Linear:
            return linear(time);   break;
    }
    return finish_;
}



void Trajectory::advanceWaypoint(const sva::PTransformd& actualPose){
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


sva::PTransformd Trajectory::quintic(double currentTime){
    double t_norm = std::min(1.0, currentTime / totalTime_);
    if (t_norm >= 1.0) {return finish_;}
        // Smooth quintic profile
              const double t2 = t_norm * t_norm;
              const double t3 = t2 * t_norm;
              const double s = t3 * (10.0 + t_norm * (-15.0 + 6.0 * t_norm));
              Eigen::Vector3d startPos = start_.translation();
              Eigen::Vector3d targetPos = finish_.translation();   
              pose_.translation() = startPos + s * (targetPos - startPos);
        // SLEPR
              Eigen::Quaterniond q_start = Eigen::Quaterniond(start_.rotation());
              Eigen::Quaterniond q_target = Eigen::Quaterniond(finish_.rotation());
              pose_.rotation() = q_start.slerp(s, q_target).toRotationMatrix();
        return pose_;
}
*/



enum class TrajectoryMode
{
    Once,
    Loop
};

class Trajectory
{
public:
    Trajectory(
        sva::PTransformd start,
        const std::vector<sva::PTransformd>& waypoints,
        double duration,
        TrajectoryMode mode,
        double positionTolerance);

    virtual ~Trajectory() = default;

    sva::PTransformd getPose() const{return pose_;}

    void update(double dt,const sva::PTransformd& actualPose);

protected:
    virtual sva::PTransformd generatePose(double time) const = 0;

    // Accesso controllato per le classi figlie
    const sva::PTransformd& start()  const  { return start_; }
    const sva::PTransformd& finish() const  { return finish_; }
    double currentTime()             const  { return currentTime_; }
    double duration()                const  { return duration_; }

private:
    bool waypointReached(const sva::PTransformd& actualPose) const;
    void advanceWaypoint(const sva::PTransformd& actualPose);

private:
    TrajectoryMode mode_;
    double positionTolerance_;

    double duration_;
    double currentTime_ = 0.0;

    std::vector<sva::PTransformd> waypoints_;
    size_t currentWaypoint_ = 0;

    sva::PTransformd start_;
    sva::PTransformd finish_;

    sva::PTransformd pose_;

    bool finished_ = false;
};

Trajectory::Trajectory(
    sva::PTransformd start,
    const std::vector<sva::PTransformd>& waypoints,
    double duration,
    TrajectoryMode mode,
    double positionTolerance)
    : mode_(mode),
      positionTolerance_(positionTolerance),
      duration_(duration),
      waypoints_(waypoints),
      start_(start),
      pose_(start)
{
    if (waypoints_.empty())
    {
        finished_ = true;
        return;
    }

    finish_ = waypoints_.front();
}

void Trajectory::update(
    double dt,
    const sva::PTransformd& actualPose)
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

bool Trajectory::waypointReached(const sva::PTransformd& actualPose) const{
    double positionError = (actualPose.translation() -finish_.translation()).norm();
    return positionError < positionTolerance_;
}

void Trajectory::advanceWaypoint(const sva::PTransformd& actualPose){
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
    Linear
};

class InterpolatedTrajectory : public Trajectory
{
public:
    InterpolatedTrajectory(
        sva::PTransformd start,
        const std::vector<sva::PTransformd>& waypoints,
        double duration,
        TrajectoryType type,
        TrajectoryMode mode,
        double positionTolerance);

protected:
    sva::PTransformd generatePose(double time) const override;

private:
    sva::PTransformd quintic(double time) const;
    sva::PTransformd spline(double time) const;
    sva::PTransformd linear(double time) const;

private:
    TrajectoryType type_;
};


InterpolatedTrajectory::InterpolatedTrajectory(
    sva::PTransformd start,
    const std::vector<sva::PTransformd>& waypoints,
    double duration,
    TrajectoryType type,
    TrajectoryMode mode,
    double positionTolerance)
    : Trajectory(
        start,
        waypoints,
        duration,
        mode,
        positionTolerance),
      type_(type)
{
}


sva::PTransformd
InterpolatedTrajectory::quintic(double time) const
{
    double t_norm = std::min(1.0, time / duration());

    if (t_norm >= 1.0)
        return finish();

    const double t2 = t_norm * t_norm;
    const double t3 = t2 * t_norm;

    const double s =
        t3 * (10.0 + t_norm * (-15.0 + 6.0 * t_norm));

    Eigen::Vector3d startPos =
        start().translation();

    Eigen::Vector3d targetPos =
        finish().translation();

    sva::PTransformd pose = start();

    pose.translation() =
        startPos + s * (targetPos - startPos);

    Eigen::Quaterniond q_start(
        start().rotation());

    Eigen::Quaterniond q_target(
        finish().rotation());

    pose.rotation() =
        q_start.slerp(s, q_target).toRotationMatrix();

    return pose;
}


sva::PTransformd
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
    }

    return finish();
}


// ===================================================================================


class OptimizedTrajectory : public Trajectory
{
public:
    OptimizedTrajectory(
        sva::PTransformd start,
        const std::vector<sva::PTransformd>& waypoints,
        double duration,
        TrajectoryMode mode,
        double positionTolerance);

protected:
    sva::PTransformd generatePose(double time) const override;
};

sva::PTransformd OptimizedTrajectory::generatePose(double time)
{
    // TODO: MPC / trajectory optimization
    return finish();
}

OptimizedTrajectory::OptimizedTrajectory(
    sva::PTransformd start,
    const std::vector<sva::PTransformd>& waypoints,
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
sva::PTransformd startPose = sva::PTransformd::Identity();

// Waypoints
sva::PTransformd waypoint1 = sva::PTransformd::Identity();
waypoint1.translation() = Eigen::Vector3d(0.5, 0.0, 0.2);

sva::PTransformd waypoint2 = sva::PTransformd::Identity();
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
// sva::PTransformd desiredPose =
//     interpolatedTrajectory.getPose();
//
// Send desiredPose to the controller / impedance task.
*/