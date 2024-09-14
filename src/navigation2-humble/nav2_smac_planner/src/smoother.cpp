// Copyright (c) 2021, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <vector>
#include <memory>
#include "nav2_smac_planner/smoother.hpp"

namespace nav2_smac_planner
{
using namespace nav2_util::geometry_utils;  // NOLINT
using namespace std::chrono;  // NOLINT

using nav2_smac_planner::common::PathPoint;
using nav2_smac_planner::common::TrajectoryPoint;
using nav2_smac_planner::common::math::Box2d;
using nav2_smac_planner::common::math::LineSegment2d;
// using nav2_smac_planner::common::math::NormalizeAngle;
using nav2_smac_planner::common::math::Vec2d;

Smoother::Smoother(const SmootherParams & params)
{
  tolerance_ = params.tolerance_;
  max_its_ = params.max_its_;
  data_w_ = params.w_data_;
  smooth_w_ = params.w_smooth_;
  is_holonomic_ = params.holonomic_;
  do_refinement_ = params.do_refinement_;
}

void Smoother::initialize(const double & min_turning_radius)
{
  min_turning_rad_ = min_turning_radius;
  state_space_ = std::make_unique<ompl::base::DubinsStateSpace>(min_turning_rad_);
}

bool Smoother::smooth(
  nav_msgs::msg::Path & path,
  nav_msgs::msg::Path & path_profile,
  const nav2_costmap_2d::Costmap2D * costmap,
  const double & max_time)
{
  // by-pass path orientations approximation when skipping smac smoother
  if (max_its_ == 0) {
    return false;
  }
  refinement_ctr_ = 0;
  steady_clock::time_point start = steady_clock::now();
  double time_remaining = max_time;
  bool success = true, reversing_segment;
  nav_msgs::msg::Path curr_path_segment;
  curr_path_segment.header = path.header;
  std::vector<PathSegment> path_segments = findDirectionalPathSegments(path);
  RCLCPP_INFO(
    rclcpp::get_logger("SmacPlannerSmoother"),
    "------path_segments.size()-----: %zu",path_segments.size());
  RCLCPP_INFO(
    rclcpp::get_logger("SmacPlannerSmoother"),
    "------path.poses.size()-----: %zu",path.poses.size());
  for (unsigned int i = 0; i != path_segments.size(); i++) {
    RCLCPP_INFO(
    rclcpp::get_logger("SmacPlannerSmoother"),
    "path_segments.size(): %zu, i: %d",path_segments.size(), i);

    if (path_segments[i].end - path_segments[i].start > 10) {
      // Populate path segment
      curr_path_segment.poses.clear();
      std::copy(
        path.poses.begin() + path_segments[i].start,
        path.poses.begin() + path_segments[i].end + 1,
        std::back_inserter(curr_path_segment.poses));

      // Make sure we're still able to smooth with time remaining
      steady_clock::time_point now = steady_clock::now();
      time_remaining = max_time - duration_cast<duration<double>>(now - start).count();

      // Smooth path segment naively
      const geometry_msgs::msg::Pose start_pose = curr_path_segment.poses.front().pose;
      const geometry_msgs::msg::Pose goal_pose = curr_path_segment.poses.back().pose;
      bool local_success =
        smoothImpl(curr_path_segment, path_profile,reversing_segment, costmap, time_remaining);
      success = success && local_success;

      // Enforce boundary conditions
      if (!is_holonomic_ && local_success) {
        enforceStartBoundaryConditions(start_pose, curr_path_segment, costmap, reversing_segment);
        enforceEndBoundaryConditions(goal_pose, curr_path_segment, costmap, reversing_segment);
      }

      // Assemble the path changes to the main path
      // std::copy(
      //   curr_path_segment.poses.begin(),
      //   curr_path_segment.poses.end(),
      //   path.poses.begin() + path_segments[i].start);
      RCLCPP_INFO(
          rclcpp::get_logger("SmacPlannerSmoother"),
                "------path_segments.size()-----");
    }
  }

  return success;
}

bool Smoother::smoothImpl(
  nav_msgs::msg::Path & path,
  nav_msgs::msg::Path & path_profile,
  bool & reversing_segment,
  const nav2_costmap_2d::Costmap2D * costmap,
  const double & max_time)
{
  
  RCLCPP_INFO(
        rclcpp::get_logger("SmacPlannerSmoother"),
          "--------------------------------------------");
          
  Eigen::MatrixXd xWS = convertPathToEigenMatrix(path);

  if (xWS.cols() < 2) {
    RCLCPP_ERROR(
        rclcpp::get_logger("SmacPlannerSmoother"),
        "reference points size smaller than two, smoother early "
              "returned");
    return false;
  }
  const auto start_timestamp = std::chrono::system_clock::now();

  // Set gear of the trajectory
  gear_ = CheckGear(xWS);

  reversing_segment = !gear_;

  DiscretizedPath warm_start_path;
  size_t xWS_size = xWS.cols();
  double accumulated_s = 0.0;
  Vec2d last_path_point(xWS(0, 0), xWS(1, 0));
  for (size_t i = 0; i < xWS_size; ++i) {
    Vec2d cur_path_point(xWS(0, i), xWS(1, i));
    accumulated_s += cur_path_point.DistanceTo(last_path_point);
    PathPoint path_point;
    path_point.set_x(xWS(0, i));
    path_point.set_y(xWS(1, i));
    path_point.set_theta(xWS(2, i));
    path_point.set_s(accumulated_s);
    warm_start_path.push_back(std::move(path_point));
    last_path_point = cur_path_point;
  }

  const double interpolated_delta_s =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .interpolated_delta_s();
  std::vector<std::pair<double, double>> interpolated_warm_start_point2ds;
  double path_length = warm_start_path.Length();
  double delta_s = path_length / std::ceil(path_length / interpolated_delta_s);

  RCLCPP_INFO(
    rclcpp::get_logger("SmacPlannerSmoother"),
    "interpolated_delta_s: %f,path_length: %f , delta_s: %f",
    interpolated_delta_s,path_length,delta_s);

  path_length += delta_s * 1.0e-6;
  for (double s = 0; s < path_length; s += delta_s) {
    const auto point2d = warm_start_path.Evaluate(s);
    interpolated_warm_start_point2ds.emplace_back(point2d.x(), point2d.y());
  }
  const size_t interpolated_size = interpolated_warm_start_point2ds.size();
  if (interpolated_size < 4) {
    RCLCPP_ERROR(
    rclcpp::get_logger("SmacPlannerSmoother"),
    "interpolated_warm_start_path smaller than 4, can't enforce "
              "heading continuity");
    return false;
  } else if (interpolated_size < 6) {
    RCLCPP_DEBUG(
        rclcpp::get_logger("SmacPlannerSmoother"),
        "interpolated_warm_start_path smaller than 4, can't enforce "
              "initial zero kappa");
    enforce_initial_kappa_ = false;
  } else {
    enforce_initial_kappa_ = true;
  }

  // Adjust heading to ensure heading continuity
  AdjustStartEndHeading(xWS, &interpolated_warm_start_point2ds);

  // 遍历 interpolated_warm_start_point2ds，生成对应的 Path 消息
  for (const auto& point : interpolated_warm_start_point2ds) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    
    // 填充PoseStamped的位置信息
    pose_stamped.pose.position.x = point.first;  // x坐标
    pose_stamped.pose.position.y = point.second; // y坐标
    pose_stamped.pose.position.z = 0.0;          // 如果z坐标不重要，可以设置为0

    // 由于我们不需要旋转信息，只需要将四元数设为单位四元数
    pose_stamped.pose.orientation.x = 0.0;
    pose_stamped.pose.orientation.y = 0.0;
    pose_stamped.pose.orientation.z = 0.0;
    pose_stamped.pose.orientation.w = 1.0;

    // 将PoseStamped消息添加到Path消息中
    path_profile.poses.push_back(pose_stamped);
  }


  // Reset path profile by discrete point heading and curvature estimation
  DiscretizedPath interpolated_warm_start_path;
  if (!SetPathProfile(interpolated_warm_start_point2ds,
                      &interpolated_warm_start_path)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("SmacPlannerSmoother"),
          "Set path profile fails ");
    return false;
  }

  // Generate feasible bounds for each path point
  std::vector<double> bounds;
  if (!GenerateInitialBounds(interpolated_warm_start_path, &bounds)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("SmacPlannerSmoother"),
          "Generate initial bounds failed, path point to close to obstaclee ");
    return false;
  }

  // Check initial path collision avoidance, if it fails, smoother assumption
  // fails. Try reanchoring

  const auto path_smooth_start_timestamp = std::chrono::system_clock::now();

  // Smooth path to have smoothed x, y, phi, kappa and s
  DiscretizedPath smoothed_path_points;
  if (!SmoothPath(interpolated_warm_start_path, bounds,
                  &smoothed_path_points)) {
    return false;
  }

  const auto path_smooth_end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> path_smooth_diff =
      path_smooth_end_timestamp - path_smooth_start_timestamp;
  RCLCPP_DEBUG(rclcpp::get_logger("SmacPlannerSmoother"),
             "iterative anchoring path smoother time: %.3f ms.",
             path_smooth_diff.count() * 1000.0);

  nav_msgs::msg::Path new_path;
  size_t num_points = smoothed_path_points.size();

  // Clear any previous path data
  new_path.poses.clear();
  new_path.poses.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
      const auto& path_point = smoothed_path_points[i];
      geometry_msgs::msg::PoseStamped pose_stamped;
      // geometry_msgs::msg::PoseStamped pose_stamped2;

      // Set x and y positions
      pose_stamped.pose.position.x = path_point.x();
      pose_stamped.pose.position.y = path_point.y();
      pose_stamped.pose.position.z = 0.0;  // z set to 0, adjust if necessary

      // Convert theta to quaternion for orientation
      double theta = path_point.theta();
      pose_stamped.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta / 2), cos(theta / 2)));

      // 
      // pose_stamped2.pose.position.x = path_point.kappa();
      // pose_stamped2.pose.position.x = path_point.dkappa();
      // pose_stamped2.pose.position.z = 0.0;
      // pose_stamped2.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta / 2), cos(theta / 2)));

      // Add the PoseStamped to the path
      new_path.poses.push_back(pose_stamped);

      // path_profile.poses.push_back(pose_stamped2);
  }
  new_path.header.frame_id = path.header.frame_id;
  new_path.header.stamp = path.header.stamp;

  path_profile.header.frame_id = path.header.frame_id;
  path_profile.header.stamp = path.header.stamp;
  
  path = new_path;

  return true;
}

double Smoother::getFieldByDim(
  const geometry_msgs::msg::PoseStamped & msg, const unsigned int & dim)
{
  if (dim == 0) {
    return msg.pose.position.x;
  } else if (dim == 1) {
    return msg.pose.position.y;
  } else {
    return msg.pose.position.z;
  }
}

void Smoother::setFieldByDim(
  geometry_msgs::msg::PoseStamped & msg, const unsigned int dim,
  const double & value)
{
  if (dim == 0) {
    msg.pose.position.x = value;
  } else if (dim == 1) {
    msg.pose.position.y = value;
  } else {
    msg.pose.position.z = value;
  }
}

std::vector<PathSegment> Smoother::findDirectionalPathSegments(const nav_msgs::msg::Path & path)
{
  std::vector<PathSegment> segments;
  PathSegment curr_segment;
  curr_segment.start = 0;

  // If holonomic, no directional changes and
  // may have abrupt angular changes from naive grid search
  if (is_holonomic_) {
    curr_segment.end = path.poses.size() - 1;
    segments.push_back(curr_segment);
    return segments;
  }

  // Iterating through the path to determine the position of the cusp
  for (unsigned int idx = 1; idx < path.poses.size() - 1; ++idx) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = path.poses[idx].pose.position.x -
      path.poses[idx - 1].pose.position.x;
    double oa_y = path.poses[idx].pose.position.y -
      path.poses[idx - 1].pose.position.y;
    double ab_x = path.poses[idx + 1].pose.position.x -
      path.poses[idx].pose.position.x;
    double ab_y = path.poses[idx + 1].pose.position.y -
      path.poses[idx].pose.position.y;

    // Checking for the existance of cusp, in the path, using the dot product.
    double dot_product = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_product < 0.0) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }

    // Checking for the existance of a differential rotation in place.
    double cur_theta = tf2::getYaw(path.poses[idx].pose.orientation);
    double next_theta = tf2::getYaw(path.poses[idx + 1].pose.orientation);
    double dtheta = angles::shortest_angular_distance(cur_theta, next_theta);
    if (fabs(ab_x) < 1e-4 && fabs(ab_y) < 1e-4 && fabs(dtheta) > 1e-4) {
      curr_segment.end = idx;
      segments.push_back(curr_segment);
      curr_segment.start = idx;
    }
  }

  curr_segment.end = path.poses.size() - 1;
  segments.push_back(curr_segment);
  return segments;
}

void Smoother::updateApproximatePathOrientations(
  nav_msgs::msg::Path & path,
  bool & reversing_segment)
{
  double dx, dy, theta, pt_yaw;
  reversing_segment = false;

  // Find if this path segment is in reverse
  dx = path.poses[2].pose.position.x - path.poses[1].pose.position.x;
  dy = path.poses[2].pose.position.y - path.poses[1].pose.position.y;
  theta = atan2(dy, dx);
  pt_yaw = tf2::getYaw(path.poses[1].pose.orientation);
  if (!is_holonomic_ && fabs(angles::shortest_angular_distance(pt_yaw, theta)) > M_PI_2) {
    reversing_segment = true;
  }

  // Find the angle relative the path position vectors
  for (unsigned int i = 0; i != path.poses.size() - 1; i++) {
    dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    theta = atan2(dy, dx);

    // If points are overlapping, pass
    if (fabs(dx) < 1e-4 && fabs(dy) < 1e-4) {
      continue;
    }

    // Flip the angle if this path segment is in reverse
    if (reversing_segment) {
      theta += M_PI;  // orientationAroundZAxis will normalize
    }

    path.poses[i].pose.orientation = orientationAroundZAxis(theta);
  }
}

unsigned int Smoother::findShortestBoundaryExpansionIdx(
  const BoundaryExpansions & boundary_expansions)
{
  // Check which is valid with the minimum integrated length such that
  // shorter end-points away that are infeasible to achieve without
  // a loop-de-loop are punished
  double min_length = 1e9;
  int shortest_boundary_expansion_idx = 1e9;
  for (unsigned int idx = 0; idx != boundary_expansions.size(); idx++) {
    if (boundary_expansions[idx].expansion_path_length<min_length &&
      !boundary_expansions[idx].in_collision &&
      boundary_expansions[idx].path_end_idx>0.0 &&
      boundary_expansions[idx].expansion_path_length > 0.0)
    {
      min_length = boundary_expansions[idx].expansion_path_length;
      shortest_boundary_expansion_idx = idx;
    }
  }

  return shortest_boundary_expansion_idx;
}

void Smoother::findBoundaryExpansion(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & end,
  BoundaryExpansion & expansion,
  const nav2_costmap_2d::Costmap2D * costmap)
{
  static ompl::base::ScopedState<> from(state_space_), to(state_space_), s(state_space_);

  from[0] = start.position.x;
  from[1] = start.position.y;
  from[2] = tf2::getYaw(start.orientation);
  to[0] = end.position.x;
  to[1] = end.position.y;
  to[2] = tf2::getYaw(end.orientation);

  double d = state_space_->distance(from(), to());
  // If this path is too long compared to the original, then this is probably
  // a loop-de-loop, treat as invalid as to not deviate too far from the original path.
  // 2.0 selected from prinicipled choice of boundary test points
  // r, 2 * r, r * PI, and 2 * PI * r. If there is a loop, it will be
  // approximately 2 * PI * r, which is 2 * PI > r, PI > 2 * r, and 2 > r * PI.
  // For all but the last backup test point, a loop would be approximately
  // 2x greater than any of the selections.
  if (d > 2.0 * expansion.original_path_length) {
    return;
  }

  std::vector<double> reals;
  double theta(0.0), x(0.0), y(0.0);
  double x_m = start.position.x;
  double y_m = start.position.y;

  // Get intermediary poses
  for (double i = 0; i <= expansion.path_end_idx; i++) {
    state_space_->interpolate(from(), to(), i / expansion.path_end_idx, s());
    reals = s.reals();
    // Make sure in range [0, 2PI)
    theta = (reals[2] < 0.0) ? (reals[2] + 2.0 * M_PI) : reals[2];
    theta = (theta > 2.0 * M_PI) ? (theta - 2.0 * M_PI) : theta;
    x = reals[0];
    y = reals[1];

    // Check for collision
    unsigned int mx, my;
    costmap->worldToMap(x, y, mx, my);
    if (static_cast<float>(costmap->getCost(mx, my)) >= INSCRIBED) {
      expansion.in_collision = true;
    }

    // Integrate path length
    expansion.expansion_path_length += hypot(x - x_m, y - y_m);
    x_m = x;
    y_m = y;

    // Store point
    expansion.pts.emplace_back(x, y, theta);
  }
}

template<typename IteratorT>
BoundaryExpansions Smoother::generateBoundaryExpansionPoints(IteratorT start, IteratorT end)
{
  std::vector<double> distances = {
    min_turning_rad_,  // Radius
    2.0 * min_turning_rad_,  // Diameter
    M_PI * min_turning_rad_,  // 50% Circumference
    2.0 * M_PI * min_turning_rad_  // Circumference
  };

  BoundaryExpansions boundary_expansions;
  boundary_expansions.resize(distances.size());
  double curr_dist = 0.0;
  double x_last = start->pose.position.x;
  double y_last = start->pose.position.y;
  geometry_msgs::msg::Point pt;
  unsigned int curr_dist_idx = 0;

  for (IteratorT iter = start; iter != end; iter++) {
    pt = iter->pose.position;
    curr_dist += hypot(pt.x - x_last, pt.y - y_last);
    x_last = pt.x;
    y_last = pt.y;

    if (curr_dist >= distances[curr_dist_idx]) {
      boundary_expansions[curr_dist_idx].path_end_idx = iter - start;
      boundary_expansions[curr_dist_idx].original_path_length = curr_dist;
      curr_dist_idx++;
    }

    if (curr_dist_idx == boundary_expansions.size()) {
      break;
    }
  }

  return boundary_expansions;
}

void Smoother::enforceStartBoundaryConditions(
  const geometry_msgs::msg::Pose & start_pose,
  nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const bool & reversing_segment)
{
  // Find range of points for testing
  BoundaryExpansions boundary_expansions =
    generateBoundaryExpansionPoints<PathIterator>(path.poses.begin(), path.poses.end());

  // Generate the motion model and metadata from start -> test points
  for (unsigned int i = 0; i != boundary_expansions.size(); i++) {
    BoundaryExpansion & expansion = boundary_expansions[i];
    if (expansion.path_end_idx == 0.0) {
      continue;
    }

    if (!reversing_segment) {
      findBoundaryExpansion(
        start_pose, path.poses[expansion.path_end_idx].pose, expansion,
        costmap);
    } else {
      findBoundaryExpansion(
        path.poses[expansion.path_end_idx].pose, start_pose, expansion,
        costmap);
    }
  }

  // Find the shortest kinematically feasible boundary expansion
  unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
  if (best_expansion_idx > boundary_expansions.size()) {
    return;
  }

  // Override values to match curve
  BoundaryExpansion & best_expansion = boundary_expansions[best_expansion_idx];
  if (reversing_segment) {
    std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
  }
  for (unsigned int i = 0; i != best_expansion.pts.size(); i++) {
    path.poses[i].pose.position.x = best_expansion.pts[i].x;
    path.poses[i].pose.position.y = best_expansion.pts[i].y;
    path.poses[i].pose.orientation = orientationAroundZAxis(best_expansion.pts[i].theta);
  }
}

void Smoother::enforceEndBoundaryConditions(
  const geometry_msgs::msg::Pose & end_pose,
  nav_msgs::msg::Path & path,
  const nav2_costmap_2d::Costmap2D * costmap,
  const bool & reversing_segment)
{
  // Find range of points for testing
  BoundaryExpansions boundary_expansions =
    generateBoundaryExpansionPoints<ReversePathIterator>(path.poses.rbegin(), path.poses.rend());

  // Generate the motion model and metadata from start -> test points
  unsigned int expansion_starting_idx;
  for (unsigned int i = 0; i != boundary_expansions.size(); i++) {
    BoundaryExpansion & expansion = boundary_expansions[i];
    if (expansion.path_end_idx == 0.0) {
      continue;
    }
    expansion_starting_idx = path.poses.size() - expansion.path_end_idx - 1;
    if (!reversing_segment) {
      findBoundaryExpansion(path.poses[expansion_starting_idx].pose, end_pose, expansion, costmap);
    } else {
      findBoundaryExpansion(end_pose, path.poses[expansion_starting_idx].pose, expansion, costmap);
    }
  }

  // Find the shortest kinematically feasible boundary expansion
  unsigned int best_expansion_idx = findShortestBoundaryExpansionIdx(boundary_expansions);
  if (best_expansion_idx > boundary_expansions.size()) {
    return;
  }

  // Override values to match curve
  BoundaryExpansion & best_expansion = boundary_expansions[best_expansion_idx];
  if (reversing_segment) {
    std::reverse(best_expansion.pts.begin(), best_expansion.pts.end());
  }
  expansion_starting_idx = path.poses.size() - best_expansion.path_end_idx - 1;
  for (unsigned int i = 0; i != best_expansion.pts.size(); i++) {
    path.poses[expansion_starting_idx + i].pose.position.x = best_expansion.pts[i].x;
    path.poses[expansion_starting_idx + i].pose.position.y = best_expansion.pts[i].y;
    path.poses[expansion_starting_idx + i].pose.orientation = orientationAroundZAxis(
      best_expansion.pts[i].theta);
  }
}
//-----------------------------个人添加-------------------------------------
// B样条函数：计算3阶B样条
std::vector<Eigen::Vector2d> Smoother::generateBSpline(const std::vector<Eigen::Vector2d> &control_points, int num_points) {
    std::vector<Eigen::Vector2d> smoothed_path;
    const int degree = 4;  // 4阶B样条
    int n = control_points.size() - 1;

    // knot向量的生成
    std::vector<double> knot(n + degree + 2);
    for (int i = 0; i <= degree; ++i) {
      knot[i] = 0.0;
      knot[n + i + 1] = 1.0;
    }
    for (int i = degree + 1; i <= n; ++i) {
      knot[i] = double(i - degree) / (n - degree + 1);
    }
    // 插值点的生成
    for (int i = 0; i < num_points; ++i) {
      double u = double(i) / (num_points - 1);
      Eigen::Vector2d point(0.0, 0.0);

      for (int j = 0; j <= n; ++j) {
        double b = basisFunction(j, degree, u, knot);
        point += b * control_points[j];
      }
        smoothed_path.push_back(point);
    }
    return smoothed_path;
}

// 基函数计算
double Smoother::basisFunction(int i, int k, double u, const std::vector<double> &knot) {
    if (k == 0) {
        return (knot[i] <= u && u < knot[i + 1]) ? 1.0 : 0.0;
    }
    double denom1 = knot[i + k] - knot[i];
    double denom2 = knot[i + k + 1] - knot[i + 1];
    double term1 = denom1 > 0 ? ((u - knot[i]) / denom1) * basisFunction(i, k - 1, u, knot) : 0;
    double term2 = denom2 > 0 ? ((knot[i + k + 1] - u) / denom2) * basisFunction(i + 1, k - 1, u, knot) : 0;
    return term1 + term2;
}

// 函数用于将nav_msgs::msg::Path中的x, y, yaw信息存放到Eigen::MatrixXd中
Eigen::MatrixXd Smoother::convertPathToEigenMatrix(const nav_msgs::msg::Path& path) {
    // 创建一个 Eigen::MatrixXd 矩阵，行数等于路径点数量，行数为3 (x, y, yaw)
    Eigen::MatrixXd matrix(3,path.poses.size());

    for (size_t i = 0; i < path.poses.size(); ++i) {
        // 获取 x 和 y
        double x = path.poses[i].pose.position.x;
        double y = path.poses[i].pose.position.y;

        // 提取 yaw
        double yaw = tf2::getYaw(path.poses[i].pose.orientation);

        // 将 x, y, yaw 存入矩阵的第 i 列
        matrix(0, i) = x;
        matrix(1, i) = y;
        matrix(2, i) = yaw;
    }
    return matrix;
  }

bool Smoother::CheckGear(const Eigen::MatrixXd& xWS) {
  // CHECK_GT(xWS.size(), 1);
  double init_heading_angle = xWS(2, 0);
  const Vec2d init_tracking_vector(xWS(0, 1) - xWS(0, 0),
                                   xWS(1, 1) - xWS(1, 0));
  double init_tracking_angle = init_tracking_vector.Angle();
  return std::abs(NormalizeAngle(init_tracking_angle - init_heading_angle)) <
         M_PI_2;
}

double Smoother::NormalizeAngle(double angle) {
    // Use fmod to get the angle in the range [-2π, 2π]
    angle = std::fmod(angle, 2.0 * M_PI);

    // If the angle is greater than π, subtract 2π to bring it back to the range [-π, π]
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }

    // If the angle is less than -π, add 2π to bring it back to the range [-π, π]
    if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    return angle;
}

void Smoother::AdjustStartEndHeading(
    const Eigen::MatrixXd& xWS,
    std::vector<std::pair<double, double>>* const point2d) {
  // Sanity check
  // CHECK_NOTNULL(point2d);
  // CHECK_GT(xWS.cols(), 1);
  // CHECK_GT(point2d->size(), 3U);

  // Set initial heading and bounds
  const double initial_heading = xWS(2, 0);
  const double end_heading = xWS(2, xWS.cols() - 1);

  // Adjust the point position to have heading by finite element difference of
  // the point and the other point equal to the given warm start initial or end
  // heading
  const double first_to_second_dx = point2d->at(1).first - point2d->at(0).first;
  const double first_to_second_dy =
      point2d->at(1).second - point2d->at(0).second;
  const double first_to_second_s =
      std::sqrt(first_to_second_dx * first_to_second_dx +
                first_to_second_dy * first_to_second_dy);
  Vec2d first_point(point2d->at(0).first, point2d->at(0).second);
  Vec2d initial_vec(first_to_second_s, 0);
  initial_vec.SelfRotate(gear_ ? initial_heading
                               : NormalizeAngle(initial_heading + M_PI));
  initial_vec += first_point;
  point2d->at(1) = std::make_pair(initial_vec.x(), initial_vec.y());

  const size_t path_size = point2d->size();
  const double second_last_to_last_dx =
      point2d->at(path_size - 1).first - point2d->at(path_size - 2).first;
  const double second_last_to_last_dy =
      point2d->at(path_size - 1).second - point2d->at(path_size - 2).second;
  const double second_last_to_last_s =
      std::sqrt(second_last_to_last_dx * second_last_to_last_dx +
                second_last_to_last_dy * second_last_to_last_dy);
  Vec2d last_point(point2d->at(path_size - 1).first,
                   point2d->at(path_size - 1).second);
  Vec2d end_vec(second_last_to_last_s, 0);
  end_vec.SelfRotate(gear_ ? NormalizeAngle(end_heading + M_PI) : end_heading);
  end_vec += last_point;
  point2d->at(path_size - 2) = std::make_pair(end_vec.x(), end_vec.y());
}

bool Smoother::SetPathProfile(
    const std::vector<std::pair<double, double>>& point2d,
    DiscretizedPath* raw_path_points) {
  if (raw_path_points == nullptr) {
    throw std::runtime_error("raw_path_points is null");
  }
  raw_path_points->clear();
  // Compute path profile
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (!DiscretePointsMath::ComputePathProfile(
          point2d, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }

  if (point2d.size() != headings.size()) {
    throw std::runtime_error("point2d and headings size mismatch");
  }
  if (point2d.size() != kappas.size()) {
      throw std::runtime_error("point2d and kappas size mismatch");
  }
  if (point2d.size() != dkappas.size()) {
      throw std::runtime_error("point2d and dkappas size mismatch");
  }
  if (point2d.size() != accumulated_s.size()) {
      throw std::runtime_error("point2d and accumulated_s size mismatch");
  }

  // Load into path point
  size_t points_size = point2d.size();
  for (size_t i = 0; i < points_size; ++i) {
    PathPoint path_point;
    path_point.set_x(point2d[i].first);
    path_point.set_y(point2d[i].second);
    path_point.set_theta(headings[i]);
    path_point.set_s(accumulated_s[i]);
    path_point.set_kappa(kappas[i]);
    path_point.set_dkappa(dkappas[i]);
    raw_path_points->push_back(std::move(path_point));
  }
  return true;
}

bool Smoother::GenerateInitialBounds(
    const DiscretizedPath& path_points, std::vector<double>* initial_bounds) {
  if (initial_bounds == nullptr) {
    throw std::runtime_error("initial_bounds is null");
  }

  initial_bounds->clear();

  const bool estimate_bound =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .estimate_bound();
  const double default_bound =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .default_bound();
  const double vehicle_shortest_dimension =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .vehicle_shortest_dimension();
  const double kEpislon = 1e-8;

  if (!estimate_bound) {
    std::vector<double> default_bounds(path_points.size(), default_bound);
    *initial_bounds = std::move(default_bounds);
    return true;
  }

  // // TODO(Jinyun): refine obstacle formulation and speed it up
  // for (const auto& path_point : path_points) {
  //   double min_bound = std::numeric_limits<double>::infinity();
  //   for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
  //     for (const LineSegment2d& linesegment : obstacle_linesegments) {
  //       min_bound =
  //           std::min(min_bound,
  //                    linesegment.DistanceTo({path_point.x(), path_point.y()}));
  //     }
  //   }
  //   min_bound -= vehicle_shortest_dimension;
  //   min_bound = min_bound < kEpislon ? 0.0 : min_bound;
  //   initial_bounds->push_back(min_bound);
  // }
  return true;
}

bool Smoother::SmoothPath(
    const DiscretizedPath& raw_path_points,
    const std::vector<double>& bounds,
    DiscretizedPath* smoothed_path_points,
    std::vector<std::vector<common::math::Vec2d>> point_box) {
  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<double> flexible_bounds;
  for (const auto& path_point : raw_path_points) {
    raw_point2d.emplace_back(path_point.x(), path_point.y());
  }
  flexible_bounds = bounds;

  FemPosDeviationSmoother fem_pos_smoother(
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .fem_pos_deviation_smoother_config());

  // TODO(Jinyun): move to confs
  const size_t max_iteration_num = 50;

  bool is_collision_free = false;
  std::vector<size_t> colliding_point_index;
  std::vector<std::pair<double, double>> smoothed_point2d;
  size_t counter = 0;

  while (!is_collision_free) {
    if (counter > max_iteration_num) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SmacPlannerSmoother"),
          "Maximum iteration reached, path smoother early stops");
      return true;
    }

    AdjustPathBounds(colliding_point_index, &flexible_bounds);

    std::vector<double> opt_x;
    std::vector<double> opt_y;
    if (!fem_pos_smoother.Solve(raw_point2d, flexible_bounds, &opt_x, &opt_y)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SmacPlannerSmoother"),
          "Smoothing path fails");
      return false;
    }

    if (opt_x.size() < 2 || opt_y.size() < 2) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SmacPlannerSmoother"),
          "Return by fem_pos_smoother is wrong. Size smaller than 2");
      return false;
    }

    // CHECK_EQ(opt_x.size(), opt_y.size());

    size_t point_size = opt_x.size();
    smoothed_point2d.clear();
    for (size_t i = 0; i < point_size; ++i) {
      smoothed_point2d.emplace_back(opt_x[i], opt_y[i]);
    }

    if (!SetPathProfile(smoothed_point2d, smoothed_path_points)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SmacPlannerSmoother"),
          "Set path profile fails");
      return false;
    }

    is_collision_free = true;
    // is_collision_free =
    //     CheckCollisionAvoidance(*smoothed_path_points, &colliding_point_index);

    RCLCPP_DEBUG(
        rclcpp::get_logger("SmacPlannerSmoother"),
          "Loop iteration number is %zu", counter);
    ++counter;
  }
  return true;
}

void Smoother::AdjustPathBounds(
    const std::vector<size_t>& colliding_point_index,
    std::vector<double>* bounds) {
  if (bounds == nullptr) {
    throw std::runtime_error("bounds is null");
  }

  const double collision_decrease_ratio =
      planner_open_space_config_.iterative_anchoring_smoother_config()
          .collision_decrease_ratio();

  for (const auto index : colliding_point_index) {
    bounds->at(index) *= collision_decrease_ratio;
  }

  // Anchor the end points to enforce the initial end end heading continuity and
  // zero kappa
  bounds->at(0) = 0.0;
  bounds->at(1) = 0.0;
  bounds->at(bounds->size() - 1) = 0.0;
  bounds->at(bounds->size() - 2) = 0.0;
  if (enforce_initial_kappa_) {
    bounds->at(2) = 0.0;
  }
}
bool Smoother::CheckCollisionAvoidance(
    const DiscretizedPath& path_points,
    std::vector<size_t>* colliding_point_index) {
  if (colliding_point_index == nullptr) {
    throw std::runtime_error("colliding_point_index is null");
  }

  colliding_point_index->clear();
  size_t path_points_size = path_points.size();
  for (size_t i = 0; i < path_points_size; ++i) {
    // Skip checking collision for thoese points colliding originally
    bool skip_checking = false;
    for (const auto index : input_colliding_point_index_) {
      if (i == index) {
        skip_checking = true;
        break;
      }
    }
    if (skip_checking) {
      continue;
    }

    const double heading = gear_
                               ? path_points[i].theta()
                               : NormalizeAngle(path_points[i].theta() + M_PI);
    Box2d ego_box(
        {path_points[i].x() + center_shift_distance_ * std::cos(heading),
         path_points[i].y() + center_shift_distance_ * std::sin(heading)},
        heading, ego_length_, ego_width_);
    
    
    bool is_colliding = false;
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const LineSegment2d& linesegment : obstacle_linesegments) {
        if (ego_box.HasOverlap(linesegment)) {
          colliding_point_index->push_back(i);
          RCLCPP_DEBUG(rclcpp::get_logger("SmacPlannerSmoother"),
             "point at %zu collided with LineSegment %s",
             i, linesegment.DebugString().c_str());
          is_colliding = true;
          break;
        }
      }
      if (is_colliding) {
        break;
      }
    }
  }

  if (!colliding_point_index->empty()) {
    return false;
  }
  return true;
}

}  // namespace nav2_smac_planner
