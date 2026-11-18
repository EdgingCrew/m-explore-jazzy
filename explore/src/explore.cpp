/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

// Navigation target for a frontier. Usually its centroid, but when the centroid
// is closer to the robot than the frontier itself (e.g. a ring-shaped frontier
// around a robot in open space), reaching it reveals nothing and the same goal
// "succeeds" forever. Then go to the frontier's closest point instead.
static geometry_msgs::msg::Point
frontierGoal(const frontier_exploration::Frontier& frontier,
             const geometry_msgs::msg::Point& robot)
{
  double dx = frontier.centroid.x - robot.x;
  double dy = frontier.centroid.y - robot.y;
  // min_distance stays infinite (middle unset) for single-cell frontiers
  if (std::isfinite(frontier.min_distance) &&
      std::hypot(dx, dy) < frontier.min_distance) {
    return frontier.middle;
  }
  return frontier.centroid;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<float>("min_unknown_area", 0.0);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<float>("coordination_scale", 0.0);
  this->declare_parameter<float>("coordination_radius", 0.0);
  this->declare_parameter<float>("completion_timeout", 0.0);
  this->declare_parameter<std::string>("robot_goals_topic", "/explore/robot_goals");
  this->declare_parameter<std::string>("blacklist_topic", "/explore/blacklist");

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("coordination_scale", coordination_scale_);
  this->get_parameter("coordination_radius", coordination_radius_);
  this->get_parameter("completion_timeout", completion_timeout_);
  std::string robot_goals_topic;
  this->get_parameter("robot_goals_topic", robot_goals_topic);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(
      costmap_client_.getCostmap(), potential_scale_, gain_scale_, min_frontier_size,
      logger_, this->get_parameter("min_unknown_area").as_double());

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Frontier goals of all robots on one shared topic. Goals are republished every
  // planning cycle, so a robot that stops publishing is forgotten after a while.
  robot_goal_timeout_ = std::max(10.0, 5.0 / planner_frequency_);
  robot_goal_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreGoal>(
      robot_goals_topic, 10);
  robot_goals_sub_ = this->create_subscription<explore_lite_msgs::msg::ExploreGoal>(
      robot_goals_topic, 10,
      std::bind(&Explore::robotGoalCallback, this, std::placeholders::_1));
  // Goals blacklisted by any robot; latched so robots started later get them too
  const auto blacklist_qos = rclcpp::QoS(100).transient_local().reliable();
  const auto blacklist_topic = this->get_parameter("blacklist_topic").as_string();
  blacklist_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      blacklist_topic, blacklist_qos);
  blacklist_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      blacklist_topic, blacklist_qos,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        // own entries are already on the list
        if (msg->header.frame_id == costmap_client_.getGlobalFrameID() &&
            !goalOnBlacklist(msg->point)) {
          frontier_blacklist_.push_back(msg->point);
        }
      });

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start exploration right away
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
  makePlan();
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers,
    const geometry_msgs::msg::Point& robot)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontierGoal(frontier, robot))) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontierGoal(frontier, robot);
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::makePlan()
{
  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  robot_position_ = pose.position;

  if (!waitForFirstGoals(pose.position)) {
    return;
  }
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    if (waitBeforeFinishing("No frontiers found")) {
      return;
    }
    RCLCPP_WARN(logger_, "No frontiers found, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  const size_t unclaimed = applyCoordination(frontiers, pose.position);

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers, pose.position);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this, &pose](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(frontierGoal(f, pose.position));
                       });
  if (frontier == frontiers.end()) {
    if (waitBeforeFinishing("All frontiers traversed/tried out")) {
      return;
    }
    RCLCPP_WARN(logger_, "All frontiers traversed/tried out, stopping.");
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }
  no_frontier_since_.reset();
  // Every usable frontier is another robot's goal: wait until one frees up instead
  // of following that robot, both would drive to the same place and block each other
  if (static_cast<size_t>(frontier - frontiers.begin()) >= unclaimed) {
    if (goal_active_) {
      move_base_client_->async_cancel_all_goals();
      goal_active_ = false;
    }
    publishRobotGoal(prev_goal_, false);
    RCLCPP_INFO_THROTTLE(logger_, *this->get_clock(), 10000,
                         "All frontiers are other robots' goals, waiting");
    return;
  }
  geometry_msgs::msg::Point target_position =
      frontierGoal(*frontier, pose.position);

  // time out if we are not making any progress
  bool same_goal = same_point(prev_goal_, target_position);

  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->min_distance;
  }
  // Moving at all (also on a detour, when the frontier doesn't get closer) keeps the
  // claim on the goal, see publishRobotGoal()
  if (!same_goal || std::hypot(pose.position.x - moved_from_.x,
                               pose.position.y - moved_from_.y) > 0.25) {
    moved_from_ = pose.position;
    last_moved_ = this->now();
  }
  publishRobotGoal(target_position, true);
  // black list if we've made no progress for a long time
  if (goal_active_ &&
      (this->now() - last_progress_ >
       tf2::durationFromSec(progress_timeout_)) &&
      !resuming_) {
    frontier_blacklist_.push_back(target_position);
    RCLCPP_DEBUG(logger_, "Adding current goal to black list");
    makePlan();
    return;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal && goal_active_) {
    return;
  }

  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  // Face the frontier: planners that plan the goal heading (e.g. Smac lattice) then
  // add no turn at the goal, and forward-facing sensors look at the new area.
  const double yaw = std::atan2(target_position.y - pose.position.y,
                                target_position.x - pose.position.x);
  goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(yaw / 2.0);
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  goal_active_ = true;
  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      [this](const NavigationGoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "Goal was REJECTED by the action server");
          goal_active_ = false;
        } else {
          active_goal_id_ = goal_handle->get_goal_id();
          RCLCPP_DEBUG(logger_, "Goal ACCEPTED, uuid: %s",
            rclcpp_action::to_string(active_goal_id_).c_str());
        }
      };

  send_goal_options.result_callback =
      [this,
       target_position](const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, target_position);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

bool Explore::waitBeforeFinishing(const std::string& reason)
{
  // A frontier can be hidden for a while, e.g. while another robot stands in a
  // doorway and is mapped as an obstacle, or while the map is still small.
  const rclcpp::Time now = this->now();
  if (!no_frontier_since_) {
    no_frontier_since_ = now;
  }
  if ((now - *no_frontier_since_).seconds() >= completion_timeout_) {
    return false;
  }
  RCLCPP_INFO_THROTTLE(logger_, *this->get_clock(), 5000,
                       "%s, finishing unless one shows up within %.0f s",
                       reason.c_str(), completion_timeout_);
  // Stand still meanwhile: the current goal's frontier is gone, and pursuing it
  // can wedge the robot (e.g. behind a post) right before it returns home
  if (goal_active_) {
    move_base_client_->async_cancel_all_goals();
    goal_active_ = false;
  }
  // nothing to claim meanwhile
  publishRobotGoal(prev_goal_, false);
  return true;
}

bool Explore::waitForFirstGoals(const geometry_msgs::msg::Point& robot)
{
  // Robots that pick their first frontier at the same time can't see each other's
  // goals yet and tend to pick the same one. So robots take turns by name: announce
  // this robot (inactive goal) in the first cycle, then start once no robot with a
  // smaller name is still choosing, or after 5 cycles at most.
  constexpr int max_startup_cycles = 5;
  if (startup_cycles_ < 0) {
    return true;
  }
  publishRobotGoal(robot, false);
  const std::string self = this->get_fully_qualified_name();
  const bool smaller_robot_choosing = std::any_of(
      other_robot_goals_.begin(), other_robot_goals_.end(),
      [&self](const auto& entry) { return entry.first < self && !entry.second.active; });
  if (startup_cycles_ == 0 ||
      (smaller_robot_choosing && startup_cycles_ < max_startup_cycles)) {
    ++startup_cycles_;
    return false;
  }
  startup_cycles_ = -1;
  return true;
}

void Explore::publishRobotGoal(const geometry_msgs::msg::Point& goal, bool active)
{
  explore_lite_msgs::msg::ExploreGoal msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = costmap_client_.getGlobalFrameID();
  msg.robot = this->get_fully_qualified_name();
  msg.goal = goal;
  msg.position = robot_position_;
  // A robot that hasn't moved 0.25 m for a while (stuck, or wiggling in place)
  // releases its frontier, so robots waiting for a free frontier can take it over
  msg.active = active &&
               (this->now() - last_moved_).seconds() < progress_timeout_ / 3.0;
  robot_goal_pub_->publish(msg);
}

void Explore::robotGoalCallback(
    const explore_lite_msgs::msg::ExploreGoal::SharedPtr msg)
{
  if (msg->robot != this->get_fully_qualified_name()) {
    other_robot_goals_[msg->robot] = *msg;
  }
}

// Distance from point to the nearest cell of frontier
static double distanceToFrontier(const frontier_exploration::Frontier& frontier,
                                 const geometry_msgs::msg::Point& point)
{
  double distance = std::hypot(frontier.centroid.x - point.x,
                               frontier.centroid.y - point.y);
  for (const auto& p : frontier.points) {
    distance = std::min(distance, std::hypot(p.x - point.x, p.y - point.y));
  }
  return distance;
}

size_t Explore::applyCoordination(
    std::vector<frontier_exploration::Frontier>& frontiers,
    const geometry_msgs::msg::Point& robot)
{
  if (coordination_scale_ <= 0.0 && coordination_radius_ <= 0.0) {
    return frontiers.size();
  }

  // Goals are only comparable in a frame shared by all robots, e.g. the merged
  // map's frame. Frame names alone can't tell two robots' "map" frames apart.
  const std::string& frame = costmap_client_.getGlobalFrameID();
  const rclcpp::Time now = this->now();
  std::vector<std::pair<std::string, geometry_msgs::msg::Point>> other_goals;
  // all other robots, also waiting ones
  std::vector<geometry_msgs::msg::Point> other_positions;
  for (auto it = other_robot_goals_.begin(); it != other_robot_goals_.end();) {
    const auto& msg = it->second;
    if ((now - rclcpp::Time(msg.header.stamp)).seconds() > robot_goal_timeout_) {
      it = other_robot_goals_.erase(it);
      continue;
    }
    if (msg.header.frame_id == frame) {
      other_positions.push_back(msg.position);
    }
    if (msg.active && msg.header.frame_id == frame) {
      other_goals.emplace_back(msg.robot, msg.goal);
    } else if (msg.active) {
      RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 10000,
                           "Ignoring goal of %s in frame '%s', exploring in '%s'",
                           msg.robot.c_str(), msg.header.frame_id.c_str(),
                           frame.c_str());
    }
    ++it;
  }
  if (other_goals.empty() && other_positions.empty()) {
    return frontiers.size();
  }

  const std::string self = this->get_fully_qualified_name();
  // A frontier near another robot's goal is claimed by it and sorted behind the
  // unclaimed ones (makePlan waits instead of using it). Each robot keeps its current frontier while it exists
  // and is unclaimed: otherwise every goal change of one robot re-ranks the
  // frontiers of the others and they keep trading targets.
  struct Scored {
    bool claimed;
    bool current;
    frontier_exploration::Frontier frontier;
  };
  std::vector<Scored> scored;
  scored.reserve(frontiers.size());
  size_t unclaimed = 0;
  for (auto& frontier : frontiers) {
    // the frontier we are already heading for
    const bool current = goal_active_ &&
                         distanceToFrontier(frontier, prev_goal_) < coordination_radius_;
    bool claimed = false;
    double distance = std::numeric_limits<double>::infinity();
    for (const auto& [robot_name, goal] : other_goals) {
      // Two robots that picked the same frontier at once: the one with the
      // smaller name keeps it, the other moves on (no swapping back and forth).
      if (current && self < robot_name) {
        continue;
      }
      const double d = distanceToFrontier(frontier, goal);
      claimed = claimed || d < coordination_radius_;
      distance = std::min(distance, d);
    }
    if (std::isfinite(distance)) {
      // metres, like the potential_scale (distance to robot) term of the cost
      frontier.cost -= coordination_scale_ * distance;
    }
    // Leave frontiers to robots that are closer to them: otherwise two robots
    // side by side can pick each other's side and cross paths
    double closest_other = std::numeric_limits<double>::infinity();
    for (const auto& position : other_positions) {
      closest_other = std::min(closest_other, distanceToFrontier(frontier, position));
    }
    if (std::isfinite(closest_other)) {
      frontier.cost += potential_scale_ * (distanceToFrontier(frontier, robot) - closest_other);
    }
    unclaimed += claimed ? 0 : 1;
    scored.push_back({claimed, current, std::move(frontier)});
  }
  std::sort(scored.begin(), scored.end(), [](const Scored& a, const Scored& b) {
    if (a.claimed != b.claimed) {
      return !a.claimed;
    }
    if (a.current != b.current) {
      return a.current;
    }
    return a.frontier.cost < b.frontier.cost;
  });
  frontiers.clear();
  for (auto& entry : scored) {
    frontiers.push_back(std::move(entry.frontier));
  }
  return unclaimed;
}

void Explore::blacklistForAll(const geometry_msgs::msg::Point& goal)
{
  frontier_blacklist_.push_back(goal);
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = costmap_client_.getGlobalFrameID();
  msg.point = goal;
  blacklist_pub_->publish(msg);
}

bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  // 0.5 m at 5 cm resolution: a frontier's goal point shifts a little as the map
  // updates, and a slightly shifted goal must stay blacklisted
  constexpr static size_t tolerace = 10;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

#ifdef NAV2_RESULT_HAS_ERROR_CODE
bool Explore::isUnreachableGoalError(uint16_t error_code)
{
  // NavigateToPose forwards the error code of the failing BT node. Only planner
  // errors that say the goal itself cannot be reached justify a blacklist.
  using PlannerResult = nav2_msgs::action::ComputePathToPose::Result;
  return error_code == PlannerResult::GOAL_OUTSIDE_MAP ||
         error_code == PlannerResult::GOAL_OCCUPIED ||
         error_code == PlannerResult::NO_VALID_PATH;
}
#endif

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal) {
  // discard stale callbacks from previously preempted goals
  if (result.goal_id != active_goal_id_) {
    return;
  }

  goal_active_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_DEBUG(logger_, "Goal was successful");
      last_progress_ = this->now();
      prev_distance_ = 0;
      // If the frontier is still there after reaching it, being there does not
      // reveal it (e.g. goal already within the goal tolerance). Without this the
      // same goal is sent and "succeeds" again forever.
      blacklistForAll(frontier_goal);
      break;
    case rclcpp_action::ResultCode::ABORTED:
#ifdef NAV2_RESULT_HAS_ERROR_CODE
      if (result.result && isUnreachableGoalError(result.result->error_code)) {
        RCLCPP_DEBUG(logger_, "Goal aborted with error_code=%d (%s) — blacklisting frontier",
                     result.result->error_code,
                     result.result->error_msg.c_str());
        blacklistForAll(frontier_goal);
      } else if (result.result && result.result->error_code != 0) {
        // Controller/TF failures are about the robot, not the frontier. Keep the
        // frontier; progress_timeout blacklists it if the robot stays stuck.
        RCLCPP_DEBUG(logger_, "Goal aborted with error_code=%d (%s) — not blacklisting frontier",
                     result.result->error_code,
                     result.result->error_msg.c_str());
      } else {
        RCLCPP_DEBUG(logger_, "Goal aborted with error_code=0 — likely a preemption, not blacklisting");
      }
#else
      // Humble: no error_code field, blacklist unconditionally on abort
      RCLCPP_DEBUG(logger_, "Goal aborted — blacklisting frontier");
      frontier_blacklist_.push_back(frontier_goal);
#endif
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");

  goal_active_ = false;
  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  exploring_timer_->cancel();
  // also called from the destructor, possibly after shutdown
  if (rclcpp::ok()) {
    publishRobotGoal(prev_goal_, false);
  }

  if (return_to_init_ && finished_exploring) {
    // The return goal replaces the current goal. Canceling first raced with it:
    // the asynchronous cancel-all could also cancel the return goal.
    returnToInitialPose();
  } else {
    move_base_client_->async_cancel_all_goals();
  }
}

void Explore::resume()
{
  resuming_ = true;
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
