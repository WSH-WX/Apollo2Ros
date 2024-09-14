#ifndef NAV2_SMAC_PLANNER__Nav2DataStructures_HPP_
#define NAV2_SMAC_PLANNER__Nav2DataStructures_HPP_


#include <string>
#include <vector>

namespace nav2_smac_planner
{
class Nav2DataStructures {
 public:
  struct SLPoint {
    double s = 0.0;
    double l = 0.0;
  };

  struct FrenetFramePoint {
    double s = 0.0;
    double l = 0.0;
    double dl = 0.0;
    double ddl = 0.0;
  };

  struct SpeedPoint {
    double s = 0.0;
    double t = 0.0;
    double v = 0.0;
    double a = 0.0;
    double da = 0.0;
  };

  struct PathPoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double s = 0.0;
    double dkappa = 0.0;
    double ddkappa = 0.0;
    std::string lane_id;
    double x_derivative = 0.0;
    double y_derivative = 0.0;
  };

  struct Path {
    std::string name;
    std::vector<PathPoint> path_point;
  };

  struct TrajectoryPoint {
    PathPoint path_point;
    double v = 0.0;
    double a = 0.0;
    double relative_time = 0.0;
    double da = 0.0;
    double steer = 0.0;
  };

  struct Trajectory {
    std::string name;
    std::vector<TrajectoryPoint> trajectory_point;
  };

  struct VehicleMotionPoint {
    TrajectoryPoint trajectory_point;
    double steer = 0.0;
  };

  struct VehicleMotion {
    std::string name;
    std::vector<VehicleMotionPoint> vehicle_motion_point;
  };

  struct GaussianInfo {
    double sigma_x = 0.0;
    double sigma_y = 0.0;
    double correlation = 0.0;
    double area_probability = 0.0;
    double ellipse_a = 0.0;
    double ellipse_b = 0.0;
    double theta_a = 0.0;
  };
};

}

#endif