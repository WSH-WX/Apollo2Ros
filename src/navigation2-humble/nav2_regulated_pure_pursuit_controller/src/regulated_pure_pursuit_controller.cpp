// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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
// limitations under the License.

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace nav2_regulated_pure_pursuit_controller
{

void RegulatedPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 0.1;
  double control_frequency = 20.0;
  goal_dist_tol_ = 0.25;  // reasonable default before first update

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist",
    rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_collision_detection",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(0.90));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_speed", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation",
    rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  base_desired_linear_vel_ = desired_linear_vel_;
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel",
    rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity",
    min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist",
    approach_velocity_scaling_dist_);
  if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
    RCLCPP_WARN(
      logger_, "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  node->get_parameter(
    plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
    max_allowed_time_to_collision_up_to_carrot_);
  node->get_parameter(
    plugin_name_ + ".use_collision_detection",
    use_collision_detection_);
  node->get_parameter(
    plugin_name_ + ".use_regulated_linear_velocity_scaling",
    use_regulated_linear_velocity_scaling_);
  node->get_parameter(
    plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    use_cost_regulated_linear_velocity_scaling_);
  node->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist_);
  node->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain_);
  node->get_parameter(
    plugin_name_ + ".inflation_cost_scaling_factor",
    inflation_cost_scaling_factor_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_radius",
    regulated_linear_scaling_min_radius_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_speed",
    regulated_linear_scaling_min_speed_);
  node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
  node->get_parameter("controller_frequency", control_frequency);
  node->get_parameter(
    plugin_name_ + ".max_robot_pose_search_dist",
    max_robot_pose_search_dist_);
  node->get_parameter(
    plugin_name_ + ".use_interpolation",
    use_interpolation_);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  control_duration_ = 1.0 / control_frequency;

  if (inflation_cost_scaling_factor_ <= 0.0) {
    RCLCPP_WARN(
      logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
      "it should be >0. Disabling cost regulated linear velocity scaling.");
    use_cost_regulated_linear_velocity_scaling_ = false;
  }

  /** Possible to drive in reverse direction if and only if
   "use_rotate_to_heading" parameter is set to false **/

  if (use_rotate_to_heading_ && allow_reversing_) {
    RCLCPP_WARN(
      logger_, "Disabling reversing. Both use_rotate_to_heading and allow_reversing "
      "parameter cannot be set to true. By default setting use_rotate_to_heading true");
    allow_reversing_ = false;
  }

  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  carrot_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);
  //---------------------------------个人添加---------------------------------
  first_curvature_ = 1.0;  // 初始化 first_curvature_
  distance_error_ = 0.0;
  lookahead_dist_publisher_ = node->create_publisher<std_msgs::msg::Float64>("lookahead_distance", 1);
  
  curvature_debug_publisher_ = node->create_publisher<geometry_msgs::msg::Point>("curvature_constraints", 1);
  robot_pose_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose_current", 1);
  pruned_path_publisher_ = node->create_publisher<nav_msgs::msg::Path>("pruned_path", 1);
  segment_path_publisher_ = node->create_publisher<nav_msgs::msg::Path>("segemnt_path", 1);
  resampled_path_publisher_ = node->create_publisher<nav_msgs::msg::Path>("resampled_path", 1);
  carrot_pose_world_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("lookahead_point_world", 1);
  curvature_publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("curvature_control", 1);
  velocities_publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("velocities_control", 1);
  tracket_erro_publisher_ = node->create_publisher<std_msgs::msg::Float64>("dist_erro_control", 1);
  xyyaw_control_erro_publisher_ = node->create_publisher<geometry_msgs::msg::Point>("xyyaw_control_erro", 1);
  path_cur_yaw_publisher_ = node->create_publisher<geometry_msgs::msg::Point>("path_cur_yaw", 1);
  PID_erro_publisher_ = node->create_publisher<geometry_msgs::msg::Point>("PID_erro", 1);
  cur2lookahead_erro_publisher_ = node->create_publisher<geometry_msgs::msg::Point>("cur2lookahead_erro", 1);
  curvature2_publisher_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("curvature2_control", 1);

  segment_plan_path_plotjuggler_ = node->create_publisher<geometry_msgs::msg::Point>("segment_plan_path_plotjuggler", 1);
  vehicle_current_pose_plotjuggler_ = node->create_publisher<geometry_msgs::msg::Point>("vehicle_current_pose_plotjuggler", 1);
  lookaheadpoint_pose_plotjuggler_ = node->create_publisher<geometry_msgs::msg::Point>("lookaheadpoint_pose_plotjuggler", 1);
  resampled_plan_path_plotjuggler_ = node->create_publisher<geometry_msgs::msg::Point>("resampled_plan_path_plotjuggler", 1);
  //--------------------------------------------------------------------------

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);

}

void RegulatedPurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  carrot_pub_.reset();
  carrot_arc_pub_.reset();
}

void RegulatedPurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  carrot_pub_->on_activate();
  carrot_arc_pub_->on_activate();

  //---------------------------------个人添加---------------------------------
  lookahead_dist_publisher_->on_activate();
  curvature_debug_publisher_->on_activate();
  robot_pose_publisher_->on_activate();
  pruned_path_publisher_->on_activate();
  segment_path_publisher_->on_activate();
  carrot_pose_world_publisher_->on_activate();
  curvature_publisher_->on_activate();
  tracket_erro_publisher_->on_activate();
  xyyaw_control_erro_publisher_->on_activate();
  path_cur_yaw_publisher_->on_activate();
  PID_erro_publisher_->on_activate();
  cur2lookahead_erro_publisher_->on_activate();
  resampled_path_publisher_->on_activate();
  velocities_publisher_->on_activate();
  curvature2_publisher_->on_activate();
  segment_plan_path_plotjuggler_ ->on_activate();
  vehicle_current_pose_plotjuggler_ ->on_activate();
  lookaheadpoint_pose_plotjuggler_->on_activate();
  resampled_plan_path_plotjuggler_->on_activate();
  //--------------------------------------------------------------------------

  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &RegulatedPurePursuitController::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void RegulatedPurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  carrot_arc_pub_->on_deactivate();
  //---------------------------------个人添加---------------------------------
  lookahead_dist_publisher_->on_deactivate();
  curvature_debug_publisher_->on_deactivate();
  robot_pose_publisher_->on_deactivate();
  pruned_path_publisher_->on_deactivate();
  segment_path_publisher_->on_deactivate();
  carrot_pose_world_publisher_->on_deactivate();
  curvature_publisher_->on_deactivate();
  tracket_erro_publisher_->on_deactivate();
  xyyaw_control_erro_publisher_->on_deactivate();
  path_cur_yaw_publisher_->on_deactivate();
  PID_erro_publisher_->on_deactivate();
  cur2lookahead_erro_publisher_->on_deactivate();
  resampled_path_publisher_->on_deactivate();
  velocities_publisher_->on_deactivate();
  curvature2_publisher_->on_deactivate();
  segment_plan_path_plotjuggler_ ->on_deactivate();
  vehicle_current_pose_plotjuggler_ ->on_deactivate();
  lookaheadpoint_pose_plotjuggler_->on_deactivate();
  resampled_plan_path_plotjuggler_->on_deactivate();
  //--------------------------------------------------------------------------
  dyn_params_handler_.reset();
}

std::unique_ptr<geometry_msgs::msg::PointStamped> RegulatedPurePursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
  }

  // // 确保前瞻距离在最小和最大范围内
  // lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);

  return fabs(1.0*speed.linear.x) + min_lookahead_dist_;

  // return lookahead_dist;
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (allow_reversing_) {
    // Cusp check
    double dist_to_cusp = findVelocitySignChange(transformed_plan);
    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }
  
  auto message = std_msgs::msg::Float64();
  message.data = lookahead_dist;
  lookahead_dist_publisher_->publish(message);
  //---------------------------------个人添加---------------------------------
  geometry_msgs::msg::PoseStamped carrot_pose;
  //如果机器人当前位置距离路径末点小于阈值，则直接将末点设置未前瞻点
  if (cur_to_end_point_dis_ < 0.1) {
    // 获取路径的末点
    geometry_msgs::msg::PoseStamped end_point = transformed_plan.poses.back();
    carrot_pose = end_point;
    // 输出距离日志
    RCLCPP_INFO(logger_,
            "Lookaheadpoint is end point");
  } else {
    // 正常计算前瞻点
    carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  }
  //------------------------------------------------------------------------

  // auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);

  //---------------------------------个人添加---------------------------------
  // 防止前瞻点来回抖动
  if (!first_run_) {
    double new_lookahead_x = carrot_pose.pose.position.x;
    double previous_lookahead_x = previous_carrot_pose_.pose.position.x;
    // 保持同向
    if ((new_lookahead_x > 0 && previous_lookahead_x < 0) || (new_lookahead_x < 0 && previous_lookahead_x) > 0) {
      // carrot_pose = previous_carrot_pose_; // 保持前瞻点不变
      size_t index = static_cast<size_t>(transformed_plan.poses.size() / 3);
      carrot_pose = transformed_plan.poses[index];
      //修正前瞻距离
      lookahead_dist = (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
                        (carrot_pose.pose.position.y * carrot_pose.pose.position.y);
      //   RCLCPP_INFO(logger_, "Randomly set the lookahead point to the end point.");
    } else {
      previous_carrot_pose_ = carrot_pose; // 更新前瞻点
    }
  } else {
    previous_carrot_pose_ = carrot_pose;
    first_run_ = false;
  }
  //-------------------------------------------------------------------------
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  //---------------------------------个人添加---------------------------------
  // 将 carrot_pose 从机器人坐标系转换为世界坐标系
  geometry_msgs::msg::PoseStamped carrot_pose_world;
  if (transformPose(global_plan_.header.frame_id, carrot_pose, carrot_pose_world)) {
    // 发布转换后的 carrot_pose
    carrot_pose_world_publisher_->publish(carrot_pose_world);

    geometry_msgs::msg::Point lookaheadpoint_pose_point;
    // 获取 x 和 y 坐标
    lookaheadpoint_pose_point.x = carrot_pose_world.pose.position.x;
    lookaheadpoint_pose_point.y = carrot_pose_world.pose.position.y;
    // 计算 yaw（航向角）
    lookaheadpoint_pose_point.z = tf2::getYaw(carrot_pose_world.pose.orientation);
    lookaheadpoint_pose_plotjuggler_->publish(lookaheadpoint_pose_point);
  } 
  //--------------------------------------------------------------------------

  double linear_vel, angular_vel;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }
  //---------------------------------个人添加--------------------------------
  // 计算车辆当前速度
  double velocity = std::sqrt(speed.linear.x * speed.linear.x + speed.linear.y * speed.linear.y);

  // 计算横向误差
  double dy = carrot_pose.pose.position.y ;

  double lateral_error = dy;// 车辆坐标系

  // 使用纯跟踪算法计算转向角
  double wheelbase_ = 0.3187;
  double maxSteeringAngle = 0.42;
  
  // double steering_angle_lateral = std::atan(2.0 * wheelbase_ * lateral_error / std::pow(lookahead_dist, 2));
  // ------double maxSteeringAngle = (23 / 180) * M_PI = 0.42;  // 最大转角限制------（准确值22.7->tan(theta = L/R)）
  double steering_angle_lateral = std::atan(2.0 * wheelbase_ * lateral_error / std::pow(lookahead_dist, 2));

  double steering_angle_lateral_origin = steering_angle_lateral;
  
  // 限制转向角在最大和最小转向角范围内
  if (steering_angle_lateral > maxSteeringAngle) {
      steering_angle_lateral = maxSteeringAngle;
  } else if (steering_angle_lateral < -maxSteeringAngle) {
      steering_angle_lateral = -maxSteeringAngle;
  }

  double angular_velocity_lateral = (velocity * std::tan(steering_angle_lateral)) / wheelbase_;

  double angular_velocity = angular_velocity_lateral;

  geometry_msgs::msg::Point point_msg3,point_msg4;
  point_msg3.x = lateral_error;
  point_msg3.y = steering_angle_lateral;
  point_msg3.z = velocity;
  PID_erro_publisher_->publish(point_msg3);

  point_msg4.x = 0.0;
  point_msg4.y = angular_velocity;
  point_msg4.z = steering_angle_lateral_origin;
  cur2lookahead_erro_publisher_->publish(point_msg4);
  //-------------------------------------------------------------------------

  // Setting the velocity direction
  double sign = 1.0;
  if (allow_reversing_) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }
  //---------------------------------个人添加---------------------------------
  // 添加启动限制
  double ramp_time = 10.0; // 2秒内加速到 desired_linear_vel_
  // 初始化 start_time_，确保只在首次调用时初始化
  if (start_time_.seconds() == 0.0) {
    start_time_ = clock_->now();  // 使用时钟对象获取当前时间
  }
  double current_time = (clock_->now() - start_time_).seconds();
  double ramped_vel = std::min(desired_linear_vel_, current_time / ramp_time * desired_linear_vel_);
  linear_vel = ramped_vel;
  //--------------------------------------------------------------------------

  // linear_vel = desired_linear_vel_;

  // Make sure we're in compliance with basic constraints
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    applyConstraints(
      curvature, speed,
      costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, sign);

    // Apply curvature to angular velocity after constraining linear velocity
    // angular_vel = linear_vel * curvature;
  }
  
  // ------------------------------个人添加--------------------------------------
  angular_vel = angular_velocity*sign;
  // ---------------------------------------------------------------------------

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (use_collision_detection_ && isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) {
    throw nav2_core::PlannerException("RegulatedPurePursuitController detected collision ahead!");
  }
  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  
  return cmd_vel;
}

bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::msg::Twist & curr_speed)
{
  // Rotate in place using max angular velocity / acceleration possible
  linear_vel = 0.0;
  const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
  angular_vel = sign * rotate_to_heading_angular_vel_;

  const double & dt = control_duration_;
  const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
  angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
}

geometry_msgs::msg::Point RegulatedPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  //--------------------------个人添加-----------------------------
  if ((r * r * dr2 - D * D) < 0) {
    // 无交点进行打印输出
    RCLCPP_INFO(
      rclcpp::get_logger("RegulatedPurePursuitController"),
      "Intersection Faile------------------------------");
  }
  //--------------------------------------------------------------

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // auto goal_pose_it = transformed_plan.poses.begin();
  // double accumulated_path_length = 0.0;

  // // 遍历路径，累加路径长度
  // for (auto it = transformed_plan.poses.begin(); it != transformed_plan.poses.end(); ++it) {
  //     if (it != transformed_plan.poses.begin()) {
  //         // 获取当前点和上一个点的坐标
  //         const auto &prev_pose = std::prev(it)->pose.position;
  //         const auto &current_pose = it->pose.position;
          
  //         // 计算相邻点之间的欧氏距离
  //         double segment_length = hypot(current_pose.x - prev_pose.x, current_pose.y - prev_pose.y);
          
  //         // 累加到路径长度
  //         accumulated_path_length += segment_length;
  //     }
      
  //     // 检查累积路径长度是否大于前瞻距离
  //     if (accumulated_path_length >= lookahead_dist) {
  //         goal_pose_it = it;
  //         break;
  //     }
  // }

  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } 
  // else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
  //   // Find the point on the line segment between the two poses
  //   // that is exactly the lookahead distance away from the robot pose (the origin)
  //   // This can be found with a closed form for the intersection of a segment and a circle
  //   // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
  //   // and goal_pose is guaranteed to be outside the circle.
  //   auto prev_pose_it = std::prev(goal_pose_it);
  //   auto point = circleSegmentIntersection(
  //     prev_pose_it->pose.position,
  //     goal_pose_it->pose.position, lookahead_dist);
  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.frame_id = prev_pose_it->header.frame_id;
  //   pose.header.stamp = goal_pose_it->header.stamp;
  //   pose.pose.position = point;
  //   return pose;
  // }

  return *goal_pose_it;
}

bool RegulatedPurePursuitController::isCollisionImminent(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const double & linear_vel, const double & angular_vel,
  const double & carrot_dist)
{
  // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
  // odom frame and the carrot_pose is in robot base frame.

  // check current point is OK
  if (inCollision(
      robot_pose.pose.position.x, robot_pose.pose.position.y,
      tf2::getYaw(robot_pose.pose.orientation)))
  {
    return true;
  }

  // visualization messages
  nav_msgs::msg::Path arc_pts_msg;
  arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  arc_pts_msg.header.stamp = robot_pose.header.stamp;
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
  pose_msg.header.stamp = arc_pts_msg.header.stamp;

  double projection_time = 0.0;
  if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01) {
    // rotating to heading at goal or toward path
    // Equation finds the angular distance required for the largest
    // part of the robot radius to move to another costmap cell:
    // theta_min = 2.0 * sin ((res/2) / r_max)
    // via isosceles triangle r_max-r_max-resolution,
    // dividing by angular_velocity gives us a timestep.
    double max_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    projection_time =
      2.0 * sin((costmap_->getResolution() / 2) / max_radius) / fabs(angular_vel);
  } else {
    // Normal path tracking
    projection_time = costmap_->getResolution() / fabs(linear_vel);
  }

  const geometry_msgs::msg::Point & robot_xy = robot_pose.pose.position;
  geometry_msgs::msg::Pose2D curr_pose;
  curr_pose.x = robot_pose.pose.position.x;
  curr_pose.y = robot_pose.pose.position.y;
  curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

  // only forward simulate within time requested
  int i = 1;
  while (i * projection_time < max_allowed_time_to_collision_up_to_carrot_) {
    i++;

    // apply velocity at curr_pose over distance
    curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
    curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
    curr_pose.theta += projection_time * angular_vel;

    // check if past carrot pose, where no longer a thoughtfully valid command
    if (hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) > carrot_dist) {
      break;
    }

    // store it for visualization
    pose_msg.pose.position.x = curr_pose.x;
    pose_msg.pose.position.y = curr_pose.y;
    pose_msg.pose.position.z = 0.01;
    arc_pts_msg.poses.push_back(pose_msg);

    // check for collision at the projected pose
    if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {
      carrot_arc_pub_->publish(arc_pts_msg);
      return true;
    }
  }

  carrot_arc_pub_->publish(arc_pts_msg);

  return false;
}

bool RegulatedPurePursuitController::inCollision(
  const double & x,
  const double & y,
  const double & theta)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 30000,
      "The dimensions of the costmap is too small to successfully check for "
      "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
      "increase your costmap size.");
    return false;
  }

  double footprint_cost = collision_checker_->footprintCostAtPose(
    x, y, theta, costmap_ros_->getRobotFootprint());
  if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
  {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return footprint_cost >= static_cast<double>(LETHAL_OBSTACLE);
}

double RegulatedPurePursuitController::costAtPose(const double & x, const double & y)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_FATAL(
      logger_,
      "The dimensions of the costmap is too small to fully include your robot's footprint, "
      "thusly the robot cannot proceed further");
    throw nav2_core::PlannerException(
            "RegulatedPurePursuitController: Dimensions of the costmap are too small "
            "to encapsulate the robot footprint at current speeds!");
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost);
}

double RegulatedPurePursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path
) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void RegulatedPurePursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path,
  double & linear_vel
) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

void RegulatedPurePursuitController::applyConstraints(
  const double & curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
  const double & pose_cost, const nav_msgs::msg::Path & path, double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel;
  double cost_vel = linear_vel;

  // limit the linear velocity by curvature
  const double radius = fabs(1.0 / curvature);
  const double & min_rad = regulated_linear_scaling_min_radius_;
  if (use_regulated_linear_velocity_scaling_ && radius < min_rad) {
    // 非线性调节函数幂函数
    double scaling_factor = pow(radius / min_rad, 2);
    curvature_vel *= scaling_factor;
    // curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
  }

  // limit the linear velocity by proximity to obstacles
  if (use_cost_regulated_linear_velocity_scaling_ &&
    pose_cost != static_cast<double>(NO_INFORMATION) &&
    pose_cost != static_cast<double>(FREE_SPACE))
  {
    const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
      std::log(pose_cost / (INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius;

    if (min_distance_to_obstacle < cost_scaling_dist_) {
      cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
    }
  }

  // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
  linear_vel = std::min(cost_vel, curvature_vel);
  linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

  //--------------------------个人添加-----------------------------
  // 创建并发布 Point 消息
  if (curvature_debug_publisher_) {
    geometry_msgs::msg::Point point_msg;
    point_msg.x = radius;
    point_msg.y = curvature_vel;
    point_msg.z = linear_vel;
    curvature_debug_publisher_->publish(point_msg);
  }
  //--------------------------------------------------------------

  applyApproachVelocityScaling(path, linear_vel);

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);
  linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO(logger_, "path_self_control path length: %zu", path.poses.size());
  global_plan_ = path;
  //--------------------------个人添加-----------------------------
  // 给曲率进行索引使用
  const_global_plan_ = path;

  // 计算路径的曲率
  curvatures_ = calculateCurvature(path);
  // 获得新路径即初始化该参数
  first_run_ = true;
  segment_path_publisher_->publish(path);
  // 创建消息并发布曲率
  std_msgs::msg::Float64MultiArray curvature_msg,velocities_msg,curvature2_msg;
  curvature_msg.data = curvatures_;
  curvature_publisher_->publish(curvature_msg);

  // 
  setResampledPath(path);
  // global_plan_ = resampled_path;

  resampled_path_publisher_->publish(resampled_path);

  //
  std::vector<double> curvatures2_ = calculateCurvature(resampled_path);
  curvature2_msg.data = curvatures2_;
  curvature2_publisher_->publish(curvature2_msg);


  velocities_ = generateVelocities(resampled_path,curvatures2_);
  velocities_msg.data = velocities_;
  velocities_publisher_->publish(velocities_msg);


  // 转存路径至point只，便于Plotjuggler显示
  // 遍历 path 中的每个轨迹点
  for (const auto &pose_stamped : path.poses) {
      geometry_msgs::msg::Point point;
      
      // 获取 x 和 y 坐标
      point.x = pose_stamped.pose.position.x;
      point.y = pose_stamped.pose.position.y;
      
      // 计算 yaw（航向角）
      point.z = tf2::getYaw(pose_stamped.pose.orientation);

      // 发布 point 数据
      segment_plan_path_plotjuggler_->publish(point);
    }

  for (const auto &resampled_stamped : resampled_path.poses) {
      geometry_msgs::msg::Point point,resampled_point;
      
      // 获取 x 和 y 坐标
      resampled_point.x = resampled_stamped.pose.position.x;
      resampled_point.y = resampled_stamped.pose.position.y;
      
      // 计算 yaw（航向角）
      resampled_point.z = tf2::getYaw(resampled_stamped.pose.orientation);

      // 发布 point 数据
      resampled_plan_path_plotjuggler_->publish(resampled_point);
    }
  //---------------------------------------------------------------
}

void RegulatedPurePursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    // Restore default value
    desired_linear_vel_ = base_desired_linear_vel_;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      desired_linear_vel_ = speed_limit;
    }
  }
}

nav_msgs::msg::Path RegulatedPurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }
  // ----------------------------个人添加-------------------------------
  // 获取路径末点
  const geometry_msgs::msg::PoseStamped &global_end_point = global_plan_.poses.back();
  // 计算机器人当前位置到路径末点的欧氏距离
  double cur_to_end_point_dis = hypot(
                          robot_pose.pose.position.x - global_end_point.pose.position.x,
                          robot_pose.pose.position.y - global_end_point.pose.position.y);
  cur_to_end_point_dis_ = cur_to_end_point_dis;
  // 发布 robot_pose
  robot_pose_publisher_->publish(robot_pose);

  geometry_msgs::msg::Point vehicle_current_point;
  // 获取 x 和 y 坐标
  vehicle_current_point.x = robot_pose.pose.position.x;
  vehicle_current_point.y = robot_pose.pose.position.y;
  
  // 计算 yaw（航向角）
  vehicle_current_point.z = tf2::getYaw(robot_pose.pose.orientation);
  vehicle_current_pose_plotjuggler_->publish(vehicle_current_point);
  // -------------------------------------------------------------------

  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();


  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose) > max_costmap_extent;
    });
  // --------------------------个人添加-----------------------------

  // Calculate distance error between the robot's current position and the closest path point
  double distance_error = euclidean_distance(robot_pose, *transformation_begin);
  distance_error_ = distance_error;

  // 计算航向误差
  // 提取机器人当前的航向角
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 提取最接近的路径点的航向角
  double closest_path_yaw = tf2::getYaw(transformation_begin->pose.orientation);
  match_point_heading = closest_path_yaw;

  // 计算航向误差
  double heading_error = angles::shortest_angular_distance(robot_yaw, closest_path_yaw);

  // 获取机器人当前位置和最近路径点的位置
  double dx = transformation_begin->pose.position.x - robot_pose.pose.position.x;
  double dy = transformation_begin->pose.position.y - robot_pose.pose.position.y;

  // 计算纵向误差和横向误差
  double longitudinal_error = dx * cos(robot_yaw) + dy * sin(robot_yaw);  // 纵向误差
  double lateral_error = -dx * sin(robot_yaw) + dy * cos(robot_yaw);      // 横向误差


  // 横、纵、航向误差发布
  geometry_msgs::msg::Point point_msg;
  point_msg.x = longitudinal_error;
  point_msg.y = lateral_error;
  point_msg.z = (heading_error/3.14)*180;
  xyyaw_control_erro_publisher_->publish(point_msg);
  // 距离误差发布
  auto message = std_msgs::msg::Float64();
  message.data = distance_error_;
  tracket_erro_publisher_->publish(message);

  // 提取 transformation_begin 到 transformation_end 之间的路径点
  nav_msgs::msg::Path extracted_path;
  for (auto it = transformation_begin; it != transformation_end; ++it) {
    extracted_path.poses.push_back(*it);
  }
  // first_curvature
  // 计算裁剪片段的第一个曲率值的索引并存储在成员变量中
  size_t first_index = 0;
  const geometry_msgs::msg::PoseStamped &target_pose = *transformation_begin;
  for (size_t i = 0; i < const_global_plan_.poses.size(); ++i) {
    if (const_global_plan_.poses[i].pose == target_pose.pose) {
      first_index = i; 
      break;
    }
  }
  first_curvature_ = curvatures_[first_index];
  
  // 设置提取路径的头信息
  extracted_path.header.frame_id = global_plan_.header.frame_id;
  extracted_path.header.stamp = global_plan_.header.stamp;

  // 发布裁剪后的路径
  pruned_path_publisher_->publish(extracted_path);
  //-----------------------------------------------------------------------

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);

  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

double RegulatedPurePursuitController::findVelocitySignChange(
  const nav_msgs::msg::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x -
      transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y -
      transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
      transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
      transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existance of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    if ( (oa_x * ab_x) + (oa_y * ab_y) < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}

bool RegulatedPurePursuitController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double RegulatedPurePursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

//----------------------------------个人添加----------------------------------
std::vector<double> RegulatedPurePursuitController::calculateCurvature(const nav_msgs::msg::Path &global_plan)
{
    std::vector<double> curvatures;
    const auto &poses = global_plan.poses;

    // 确保路径点数目足够计算曲率（至少3个点）
    if (poses.size() < 3) {
        throw std::runtime_error("Path is too short to calculate curvature.");
    }

    for (size_t i = 1; i < poses.size() - 1; ++i) {
        const auto &P1 = poses[i - 1].pose.position;
        const auto &P2 = poses[i].pose.position;
        const auto &P3 = poses[i + 1].pose.position;

        double x1 = P1.x, y1 = P1.y;
        double x2 = P2.x, y2 = P2.y;
        double x3 = P3.x, y3 = P3.y;

        double numerator = std::abs((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1));
        double denominator = std::sqrt(
            std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2)) *
            std::sqrt(std::pow((x3 - x2), 2) + std::pow((y3 - y2), 2)) *
            std::sqrt(std::pow((x3 - x1), 2) + std::pow((y3 - y1), 2));
        double curvature = 0.0;
        // 避免除以零的情况
        if (denominator == 0.0) {
            curvatures.push_back(0.0);
        } else {
            double curvature = 2.0 * numerator / denominator;
            curvatures.push_back(curvature);
        }
        double yaw = tf2::getYaw(poses[i].pose.orientation);

        geometry_msgs::msg::Point point_msg;
        point_msg.x = yaw;
        point_msg.y = curvature;
        point_msg.z = 0.0;

        path_cur_yaw_publisher_->publish(point_msg);
    }

    // 第一个点和最后一个点的曲率通常不计算，可以使用第二个和倒数第二个点的曲率填充
    curvatures.insert(curvatures.begin(), curvatures.front());
    curvatures.push_back(curvatures.back());

    return curvatures;
}

void RegulatedPurePursuitController::setResampledPath(const nav_msgs::msg::Path & path)
{
  // Step 1: Initialize the resampled path
  resampled_path.header = path.header;
  resampled_path.poses.clear();

  // Step 2: Calculate total arc length of the path
  std::vector<double> arclengths;
  arclengths.push_back(0.0);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto &prev = path.poses[i - 1].pose.position;
    const auto &curr = path.poses[i].pose.position;
    double dx = curr.x - prev.x;
    double dy = curr.y - prev.y;
    double dist = std::hypot(dx, dy);
    arclengths.push_back(arclengths.back() + dist);
  }

  // Step 3: Generate new resampled arc lengths
  std::vector<double> out_arclength;
  for (double s = 0; s < arclengths.back(); s += resampling_ds) {
    out_arclength.push_back(s);
  }

  // Step 4: Interpolate poses at new arc lengths
  for (const double &s : out_arclength) {
    // Find the segment of the path where the current arc length 's' lies
    auto lower = std::lower_bound(arclengths.begin(), arclengths.end(), s);
    size_t idx = std::distance(arclengths.begin(), lower);
    if (idx == 0) {
      resampled_path.poses.push_back(path.poses[0]);
    } else if (idx >= arclengths.size()) {
      resampled_path.poses.push_back(path.poses.back());
    } else {
      // Interpolate between two poses
      const double alpha = (s - arclengths[idx - 1]) / (arclengths[idx] - arclengths[idx - 1]);
      resampled_path.poses.push_back(interpolatePose(path.poses[idx - 1].pose, path.poses[idx].pose, alpha));
    }
  }

  // Step 5: Ensure the last point matches the end of the original path
  if (!path.poses.empty()) {
    resampled_path.poses.back() = path.poses.back();
  }
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::interpolatePose(
  const geometry_msgs::msg::Pose &p1, const geometry_msgs::msg::Pose &p2, double alpha)
{
  geometry_msgs::msg::PoseStamped interpolated_pose;
  
  // Interpolate positions
  interpolated_pose.pose.position.x = p1.position.x + alpha * (p2.position.x - p1.position.x);
  interpolated_pose.pose.position.y = p1.position.y + alpha * (p2.position.y - p1.position.y);
  interpolated_pose.pose.position.z = p1.position.z + alpha * (p2.position.z - p1.position.z);

  // Interpolate orientations (using spherical linear interpolation)
  tf2::Quaternion q1, q2;
  tf2::fromMsg(p1.orientation, q1);
  tf2::fromMsg(p2.orientation, q2);
  tf2::Quaternion q_interpolated = q1.slerp(q2, alpha);
  interpolated_pose.pose.orientation = tf2::toMsg(q_interpolated);

  return interpolated_pose;
}

std::vector<double> RegulatedPurePursuitController::generateVelocities(const nav_msgs::msg::Path& plan, const std::vector<double>& curvatures) {
    size_t n = plan.poses.size();
    std::vector<double> velocities(n, 0.0);
    double v_max = 1.0; // 最大速度
    double v_min = 0.0; // 最小速度
    double k_factor = 1.0; // 曲率影响因子

    // 中间点的速度根据曲率调整
    for (size_t i = 1; i < curvatures.size(); ++i) {
        double curvature = curvatures[i - 1];
        double velocity = v_max / (1.0 + k_factor * curvature);
        if (velocity < v_min) {
            velocity = v_min;
        }
        velocities[i] = velocity;
    }

    // 平滑速度，确保起点逐渐加速，终点逐渐减速
    for (size_t i = 1; i < n - 1; ++i) {
        double scale_factor = std::sin(M_PI * static_cast<double>(i) / static_cast<double>(n - 1));
        velocities[i] *= scale_factor;
    }

    return velocities;
}

// ---------------------------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult
RegulatedPurePursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".inflation_cost_scaling_factor") {
        if (parameter.as_double() <= 0.0) {
          RCLCPP_WARN(
            logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
            "it should be >0. Ignoring parameter update.");
          continue;
        }
        inflation_cost_scaling_factor_ = parameter.as_double();
      } else if (name == plugin_name_ + ".desired_linear_vel") {
        desired_linear_vel_ = parameter.as_double();
        base_desired_linear_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_angular_vel") {
        rotate_to_heading_angular_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot") {
        max_allowed_time_to_collision_up_to_carrot_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cost_scaling_dist") {
        cost_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cost_scaling_gain") {
        cost_scaling_gain_ = parameter.as_double();
      } else if (name == plugin_name_ + ".regulated_linear_scaling_min_radius") {
        regulated_linear_scaling_min_radius_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".regulated_linear_scaling_min_speed") {
        regulated_linear_scaling_min_speed_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_angular_accel") {
        max_angular_accel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_min_angle") {
        rotate_to_heading_min_angle_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_regulated_linear_velocity_scaling") {
        use_regulated_linear_velocity_scaling_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_cost_regulated_linear_velocity_scaling") {
        use_cost_regulated_linear_velocity_scaling_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_rotate_to_heading") {
        if (parameter.as_bool() && allow_reversing_) {
          RCLCPP_WARN(
            logger_, "Both use_rotate_to_heading and allow_reversing "
            "parameter cannot be set to true. Rejecting parameter update.");
          continue;
        }
        use_rotate_to_heading_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".allow_reversing") {
        if (use_rotate_to_heading_ && parameter.as_bool()) {
          RCLCPP_WARN(
            logger_, "Both use_rotate_to_heading and allow_reversing "
            "parameter cannot be set to true. Rejecting parameter update.");
          continue;
        }
        allow_reversing_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_regulated_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController,
  nav2_core::Controller)
