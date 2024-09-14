
#pragma once

#include "pnc_point.h"

namespace nav2_smac_planner {
namespace common {

// SLPoint implementation
SLPoint::SLPoint() : s_(0), l_(0) {}

double SLPoint::s() const { return s_; }
void SLPoint::set_s(double s) { s_ = s; }

double SLPoint::l() const { return l_; }
void SLPoint::set_l(double l) { l_ = l; }

// FrenetFramePoint implementation
FrenetFramePoint::FrenetFramePoint() : s_(0), l_(0), dl_(0), ddl_(0) {}

double FrenetFramePoint::s() const { return s_; }
void FrenetFramePoint::set_s(double s) { s_ = s; }

double FrenetFramePoint::l() const { return l_; }
void FrenetFramePoint::set_l(double l) { l_ = l; }

double FrenetFramePoint::dl() const { return dl_; }
void FrenetFramePoint::set_dl(double dl) { dl_ = dl; }

double FrenetFramePoint::ddl() const { return ddl_; }
void FrenetFramePoint::set_ddl(double ddl) { ddl_ = ddl; }

// SpeedPoint implementation
SpeedPoint::SpeedPoint() : s_(0), t_(0), v_(0), a_(0), da_(0) {}

double SpeedPoint::s() const { return s_; }
void SpeedPoint::set_s(double s) { s_ = s; }

double SpeedPoint::t() const { return t_; }
void SpeedPoint::set_t(double t) { t_ = t; }

double SpeedPoint::v() const { return v_; }
void SpeedPoint::set_v(double v) { v_ = v; }

double SpeedPoint::a() const { return a_; }
void SpeedPoint::set_a(double a) { a_ = a; }

double SpeedPoint::da() const { return da_; }
void SpeedPoint::set_da(double da) { da_ = da; }

// PathPoint implementation
PathPoint::PathPoint() 
    : x_(0), y_(0), z_(0), theta_(0), kappa_(0), s_(0), dkappa_(0), ddkappa_(0),
      lane_id_(""), x_derivative_(0), y_derivative_(0) {}

double PathPoint::x() const { return x_; }
void PathPoint::set_x(double x) { x_ = x; }

double PathPoint::y() const { return y_; }
void PathPoint::set_y(double y) { y_ = y; }

double PathPoint::z() const { return z_; }
void PathPoint::set_z(double z) { z_ = z; }

double PathPoint::theta() const { return theta_; }
void PathPoint::set_theta(double theta) { theta_ = theta; }

double PathPoint::kappa() const { return kappa_; }
void PathPoint::set_kappa(double kappa) { kappa_ = kappa; }

double PathPoint::s() const { return s_; }
void PathPoint::set_s(double s) { s_ = s; }

double PathPoint::dkappa() const { return dkappa_; }
void PathPoint::set_dkappa(double dkappa) { dkappa_ = dkappa; }

double PathPoint::ddkappa() const { return ddkappa_; }
void PathPoint::set_ddkappa(double ddkappa) { ddkappa_ = ddkappa; }

const std::string& PathPoint::lane_id() const { return lane_id_; }
void PathPoint::set_lane_id(const std::string& lane_id) { lane_id_ = lane_id; }

double PathPoint::x_derivative() const { return x_derivative_; }
void PathPoint::set_x_derivative(double x_derivative) { x_derivative_ = x_derivative; }

double PathPoint::y_derivative() const { return y_derivative_; }
void PathPoint::set_y_derivative(double y_derivative) { y_derivative_ = y_derivative; }

// Path implementation
Path::Path() : name_(""), has_name_(false) {}

const std::string& Path::name() const { return name_; }
void Path::set_name(const std::string& name) {
  name_ = name;
  has_name_ = !name.empty();
}

bool Path::has_name() const { return has_name_; }

int Path::path_point_size() const { return path_point_.size(); }

const PathPoint& Path::path_point(int index) const {
  return path_point_.at(index);  // 使用 at 可以检查越界
}

PathPoint* Path::mutable_path_point(int index) {
  return &path_point_.at(index);  // 返回指针，允许修改
}

void Path::add_path_point(const PathPoint& point) {
  path_point_.push_back(point);
}

// GaussianInfo implementation
GaussianInfo::GaussianInfo()
    : sigma_x_(0), sigma_y_(0), correlation_(0), area_probability_(0), ellipse_a_(0), ellipse_b_(0), theta_a_(0) {}

double GaussianInfo::sigma_x() const { return sigma_x_; }
void GaussianInfo::set_sigma_x(double sigma_x) { sigma_x_ = sigma_x; }

double GaussianInfo::sigma_y() const { return sigma_y_; }
void GaussianInfo::set_sigma_y(double sigma_y) { sigma_y_ = sigma_y; }

double GaussianInfo::correlation() const { return correlation_; }
void GaussianInfo::set_correlation(double correlation) { correlation_ = correlation; }

double GaussianInfo::area_probability() const { return area_probability_; }
void GaussianInfo::set_area_probability(double area_probability) { area_probability_ = area_probability; }

double GaussianInfo::ellipse_a() const { return ellipse_a_; }
void GaussianInfo::set_ellipse_a(double ellipse_a) { ellipse_a_ = ellipse_a; }

double GaussianInfo::ellipse_b() const { return ellipse_b_; }
void GaussianInfo::set_ellipse_b(double ellipse_b) { ellipse_b_ = ellipse_b; }

double GaussianInfo::theta_a() const { return theta_a_; }
void GaussianInfo::set_theta_a(double theta_a) { theta_a_ = theta_a; }

// TrajectoryPoint implementation
TrajectoryPoint::TrajectoryPoint()
    : v_(0), a_(0), relative_time_(0), da_(0), steer_(0) {}

const PathPoint& TrajectoryPoint::path_point() const { return path_point_; }
void TrajectoryPoint::set_path_point(const PathPoint& point) { path_point_ = point; }

double TrajectoryPoint::v() const { return v_; }
void TrajectoryPoint::set_v(double v) { v_ = v; }

double TrajectoryPoint::a() const { return a_; }
void TrajectoryPoint::set_a(double a) { a_ = a; }

double TrajectoryPoint::relative_time() const { return relative_time_; }
void TrajectoryPoint::set_relative_time(double relative_time) { relative_time_ = relative_time; }

double TrajectoryPoint::da() const { return da_; }
void TrajectoryPoint::set_da(double da) { da_ = da; }

double TrajectoryPoint::steer() const { return steer_; }
void TrajectoryPoint::set_steer(double steer) { steer_ = steer; }

const GaussianInfo& TrajectoryPoint::gaussian_info() const { return gaussian_info_; }
void TrajectoryPoint::set_gaussian_info(const GaussianInfo& info) { gaussian_info_ = info; }

// Trajectory implementation
Trajectory::Trajectory() : name_(""), has_name_(false) {}

const std::string& Trajectory::name() const { return name_; }
void Trajectory::set_name(const std::string& name) {
  name_ = name;
  has_name_ = !name.empty();
}

bool Trajectory::has_name() const { return has_name_; }

int Trajectory::trajectory_point_size() const { return trajectory_point_.size(); }

const TrajectoryPoint& Trajectory::trajectory_point(int index) const {
  return trajectory_point_.at(index);  // 使用 at 进行边界检查
}

TrajectoryPoint* Trajectory::mutable_trajectory_point(int index) {
  return &trajectory_point_.at(index);
}

void Trajectory::add_trajectory_point(const TrajectoryPoint& point) {
  trajectory_point_.push_back(point);
}

// VehicleMotionPoint implementation
VehicleMotionPoint::VehicleMotionPoint() : steer_(0) {}

const TrajectoryPoint& VehicleMotionPoint::trajectory_point() const { return trajectory_point_; }
void VehicleMotionPoint::set_trajectory_point(const TrajectoryPoint& point) { trajectory_point_ = point; }

double VehicleMotionPoint::steer() const { return steer_; }
void VehicleMotionPoint::set_steer(double steer) { steer_ = steer; }

}
}