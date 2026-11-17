/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez.
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

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <map_merge/map_merge.h>
#include <map_merge/ros1_names.hpp>
#include <rcpputils/asserts.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
geometry_msgs::msg::Transform scaleTranslationMetersToCells(
  const geometry_msgs::msg::Transform& t, double resolution)
{
  if (resolution <= 0.0) {
    return t;
  }
  geometry_msgs::msg::Transform s = t;
  s.translation.x /= resolution;
  s.translation.y /= resolution;
  s.translation.z /= resolution;
  return s;
}

geometry_msgs::msg::Transform identityTransform()
{
  geometry_msgs::msg::Transform id;
  id.rotation.w = 1.0;
  return id;
}

}  // namespace

namespace map_merge
{
MapMerge::MapMerge() : Node("map_merge", rclcpp::NodeOptions()
                                       .allow_undeclared_parameters(true)
                                       .automatically_declare_parameters_from_overrides(true)),
subscriptions_size_(0),
logger_(this->get_logger())
{
  std::string frame_id;
  std::string merged_map_topic;

  if (!this->has_parameter("merging_rate")) this->declare_parameter<double>("merging_rate", 4.0);
  if (!this->has_parameter("discovery_rate")) this->declare_parameter<double>("discovery_rate", 0.05);
  if (!this->has_parameter("estimation_rate")) this->declare_parameter<double>("estimation_rate", 0.5);
  if (!this->has_parameter("known_init_poses")) this->declare_parameter<bool>("known_init_poses", true);
  if (!this->has_parameter("estimation_confidence")) this->declare_parameter<double>("estimation_confidence", 1.0);
  if (!this->has_parameter("robot_map_topic")) this->declare_parameter<std::string>("robot_map_topic", "map");
  if (!this->has_parameter("robot_map_updates_topic")) this->declare_parameter<std::string>("robot_map_updates_topic", "map_updates");
  if (!this->has_parameter("robot_namespace")) this->declare_parameter<std::string>("robot_namespace", "");
  if (!this->has_parameter("merged_map_topic")) this->declare_parameter<std::string>("merged_map_topic", "map");
  if (!this->has_parameter("world_frame")) this->declare_parameter<std::string>("world_frame", "world");
  if (!this->has_parameter("expand_slam_maps_to_common_canvas")) this->declare_parameter<bool>("expand_slam_maps_to_common_canvas", true);
  if (!this->has_parameter("expand_slam_maps_margin_m")) this->declare_parameter<double>("expand_slam_maps_margin_m", 1.0);
  if (!this->has_parameter("expand_slam_maps_apply_init_pose")) this->declare_parameter<bool>("expand_slam_maps_apply_init_pose", true);
  if (!this->has_parameter("publish_robot_map_tf")) this->declare_parameter<bool>("publish_robot_map_tf", false);
  if (!this->has_parameter("robot_map_frame")) this->declare_parameter<std::string>("robot_map_frame", "map");
  if (!this->has_parameter("publish_robot_maps")) this->declare_parameter<bool>("publish_robot_maps", false);
  if (!this->has_parameter("robot_merged_map_topic")) this->declare_parameter<std::string>("robot_merged_map_topic", "merged_map");

  this->get_parameter("merging_rate", merging_rate_);
  this->get_parameter("discovery_rate", discovery_rate_);
  this->get_parameter("estimation_rate", estimation_rate_);
  this->get_parameter("known_init_poses", have_initial_poses_);
  this->get_parameter("estimation_confidence", confidence_threshold_);
  this->get_parameter("robot_map_topic", robot_map_topic_);
  this->get_parameter("robot_map_updates_topic", robot_map_updates_topic_);
  this->get_parameter("robot_namespace", robot_namespace_);
  this->get_parameter("merged_map_topic", merged_map_topic);
  this->get_parameter("world_frame", world_frame_);
  this->get_parameter("expand_slam_maps_to_common_canvas", expand_slam_maps_to_common_canvas_);
  this->get_parameter("expand_slam_maps_margin_m", expand_slam_maps_margin_m_);
  this->get_parameter("expand_slam_maps_apply_init_pose", expand_slam_maps_apply_init_pose_);
  this->get_parameter("publish_robot_map_tf", publish_robot_map_tf_);
  this->get_parameter("robot_map_frame", robot_map_frame_);
  this->get_parameter("publish_robot_maps", publish_robot_maps_);
  this->get_parameter("robot_merged_map_topic", robot_merged_map_topic_);

  /* publishing */
  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  merged_map_publisher_ =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>(merged_map_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Timers
  map_merging_timer_ = this->create_wall_timer(
    std::chrono::milliseconds((uint16_t)(1000.0 / merging_rate_)),
    [this]() { mapMerging(); });
  // execute right away to simulate the ros1 first while loop on a thread
  mapMerging();

  topic_subscribing_timer_ = this->create_wall_timer(
    std::chrono::milliseconds((uint16_t)(1000.0 / discovery_rate_)),
    [this]() { topicSubscribing(); });

  // For topicSubscribing() we need to spin briefly for the discovery to happen
  rclcpp::Rate r(100);
  int i = 0;
  while (rclcpp::ok() && i < 100) {
    rclcpp::spin_some(this->get_node_base_interface());
    r.sleep();
    i++;
  }
  topicSubscribing();

  if (!have_initial_poses_){
    pose_estimation_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / estimation_rate_)),
      [this]() { poseEstimation(); });
    // execute right away to simulate the ros1 first while loop on a thread
    poseEstimation();
  }
}

/*
 * Subscribe to pose and map topics
 */
void MapMerge::topicSubscribing()
{
  RCLCPP_DEBUG(logger_, "Robot discovery started.");
  RCLCPP_INFO_ONCE(logger_, "Robot discovery started.");

  // ros::master::V_TopicInfo topic_infos;
  geometry_msgs::msg::Transform init_pose;
  std::string robot_name;
  std::string map_topic;
  std::string map_updates_topic;

  // ros::master::getTopics(topic_infos);
  std::map<std::string, std::vector<std::string>> topic_infos = this->get_topic_names_and_types();

  for (const auto& topic_it : topic_infos) {
    std::string topic_name = topic_it.first;
    std::vector<std::string> topic_types = topic_it.second;
    // iterate over all topic types
    for (const auto& topic_type : topic_types) {
      // RCLCPP_INFO(logger_, "Topic: %s, type: %s", topic_name.c_str(), topic_type.c_str());

      // we check only map topic
      if (!isRobotMapTopic(topic_name, topic_type)) {
        continue;
      }

      robot_name = robotNameFromTopic(topic_name);
      if (robots_.count(robot_name)) {
        // we already know this robot
        continue;
      }

      if (have_initial_poses_ && !getInitPose(robot_name, init_pose)) {
        RCLCPP_WARN(logger_, "Couldn't get initial position for robot [%s]\n"
                "did you defined parameters map_merge/init_pose_[xyz]? in robot "
                "namespace? If you want to run merging without known initial "
                "positions of robots please set `known_init_poses` parameter "
                "to false. See relevant documentation for details.",
                robot_name.c_str());
        continue;
      }

      RCLCPP_INFO(logger_, "adding robot [%s] to system", robot_name.c_str());
      {
        // We don't lock since because of ROS2 default executor only a callback can run at a time
        // std::lock_guard<boost::shared_mutex> lock(subscriptions_mutex_);
        subscriptions_.emplace_front();
        ++subscriptions_size_;
      }

      // no locking here. robots_ are used only in this procedure
      MapSubscription& subscription = subscriptions_.front();
      robots_.insert({robot_name, &subscription});
      subscription.robot_name = robot_name;
      subscription.initial_pose = init_pose;

      /* subscribe callbacks */
      map_topic = ros1_names::append(robot_name, robot_map_topic_);
      map_updates_topic =
          ros1_names::append(robot_name, robot_map_updates_topic_);
      RCLCPP_INFO(logger_, "Subscribing to MAP topic: %s.", map_topic.c_str());
      auto map_qos = rclcpp::QoS(rclcpp::KeepLast(50)).transient_local().reliable();
      subscription.map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          map_topic, map_qos,
          [this, &subscription](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            fullMapUpdate(msg, subscription);
          });
      RCLCPP_INFO(logger_, "Subscribing to MAP updates topic: %s.",
              map_updates_topic.c_str());
      subscription.map_updates_sub =
          this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
              map_updates_topic, map_qos,
              [this, &subscription](
                  const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
                partialMapUpdate(msg, subscription);
              });

      // only the expansion path places the merged map in world_frame by init_pose
      if (have_initial_poses_ && expand_slam_maps_to_common_canvas_) {
        if (publish_robot_map_tf_) {
          publishRobotMapTf(subscription);
        }
        if (publish_robot_maps_) {
          subscription.robot_merged_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
              ros1_names::append(robot_name, robot_merged_map_topic_),
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        }
      }
    }
  }
}

/*
 * mapMerging()
 */
void MapMerge::mapMerging()
{
  RCLCPP_DEBUG(logger_, "Map merging started.");
  RCLCPP_INFO_ONCE(logger_, "Map merging started.");

  nav_msgs::msg::OccupancyGrid::SharedPtr merged_map;
  // filled on the expansion path: robots and their maps on the merged canvas
  std::vector<MapSubscription*> merged_robots;
  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> grids;
  bool did_expand = false;

  if (have_initial_poses_) {
    /* Sort by robot name so reference order is stable (e.g. robot1 first; matches init_pose convention). */
    std::vector<MapSubscription*> sorted;
    sorted.reserve(subscriptions_size_);
    for (auto& subscription : subscriptions_) {
      sorted.push_back(&subscription);
    }
    std::sort(sorted.begin(), sorted.end(),
              [](const MapSubscription* a, const MapSubscription* b) {
                return a->robot_name < b->robot_name;
              });

    std::vector<geometry_msgs::msg::Transform> transforms;
    grids.reserve(sorted.size());
    transforms.reserve(sorted.size());
    for (const auto* subscription : sorted) {
      grids.push_back(subscription->readonly_map);
      transforms.push_back(subscription->initial_pose);
    }

    geometry_msgs::msg::Pose canvas_origin;
    if (expand_slam_maps_to_common_canvas_) {
      /* init_pose_* is applied while embedding, where cells have metric positions and the
       * rotation pivots about each robot's map origin. The OpenCV merge below then only
       * overlays. (Its warp applies a transform inverted and pivots about the grid corner.) */
      const std::vector<geometry_msgs::msg::Transform> map_poses =
        expand_slam_maps_apply_init_pose_
          ? transforms
          : std::vector<geometry_msgs::msg::Transform>(transforms.size(), identityTransform());
      std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> expanded;
      if (expandSlamMapsToCommonCanvas(grids, map_poses, expanded) &&
          expanded.size() == grids.size()) {
        grids = std::move(expanded);
        did_expand = true;
        merged_robots = sorted;
        canvas_origin = grids.front()->info.origin;
        RCLCPP_INFO_ONCE(
          logger_, "SLAM map expansion: embedded all robot maps in one common grid (%s)",
          expand_slam_maps_apply_init_pose_ ? "placed by init_pose_*"
                                            : "map frames assumed to coincide, init_pose_* ignored");
      }
    }

    double resolution = 0.0;
    for (const auto& g : grids) {
      if (g && g->info.resolution > 0.f) {
        resolution = static_cast<double>(g->info.resolution);
        break;
      }
    }
    std::vector<geometry_msgs::msg::Transform> transforms_cells;
    transforms_cells.reserve(transforms.size());
    if (did_expand) {
      transforms_cells.assign(transforms.size(), identityTransform());
    } else {
      /* Legacy path: init_pose only. */
      for (const auto& t : transforms) {
        transforms_cells.push_back(scaleTranslationMetersToCells(t, resolution));
      }
    }

    pipeline_.feed(grids.begin(), grids.end());
    pipeline_.setTransforms(transforms_cells.begin(), transforms_cells.end());
    {
      std::lock_guard<std::mutex> lock(pipeline_mutex_);
      merged_map = pipeline_.composeGrids(logger_);
    }
    // identity overlay of equal canvases: the merged grid is the canvas, so it has
    // a real position in world_frame (composeGrids centres it otherwise)
    if (merged_map && did_expand) {
      merged_map->info.origin = canvas_origin;
    }
  } else {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    merged_map = pipeline_.composeGrids(logger_);
  }

  if (!merged_map) {
    return;
  }

  RCLCPP_DEBUG(logger_, "all maps merged, publishing");
  // RCLCPP_INFO(logger_, "all maps merged, publishing");
  auto now = this->now();
  merged_map->info.map_load_time = now;
  merged_map->header.stamp = now;
  merged_map->header.frame_id = world_frame_;

  rcpputils::assert_true(merged_map->info.resolution > 0.f);
  merged_map_publisher_->publish(*merged_map);

  if (publish_robot_maps_ && did_expand) {
    publishRobotMaps(*merged_map, grids, merged_robots);
  }
}

void MapMerge::publishRobotMaps(
  const nav_msgs::msg::OccupancyGrid& merged,
  const std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr>& own_maps,
  const std::vector<MapSubscription*>& robots)
{
  for (size_t i = 0; i < robots.size(); ++i) {
    const auto& own = own_maps[i];
    if (!robots[i]->robot_merged_map_pub || !own) {
      continue;
    }
    // own maps were expanded to the merge canvas, which the merged map is
    if (own->data.size() != merged.data.size()) {
      RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 5000,
                           "Merged map is not the expansion canvas, not publishing robot maps");
      return;
    }
    nav_msgs::msg::OccupancyGrid robot_map = merged;
    for (size_t c = 0; c < robot_map.data.size(); ++c) {
      if (own->data[c] >= 0) {
        robot_map.data[c] = own->data[c];
      }
    }
    robots[i]->robot_merged_map_pub->publish(robot_map);
  }
}

bool MapMerge::expandSlamMapsToCommonCanvas(
  const std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr>& in,
  const std::vector<geometry_msgs::msg::Transform>& map_poses,
  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr>& out)
{
  out.clear();
  /* Each valid grid with the transform from its cell coordinates (cells * resolution)
   * to the merge frame: pose of the robot's map frame in the merge frame * map.info.origin. */
  std::vector<std::pair<const nav_msgs::msg::OccupancyGrid*, tf2::Transform>> valid;
  for (size_t i = 0; i < in.size(); ++i) {
    const auto& g = in[i];
    if (g && !g->data.empty() && g->info.resolution > 0.f) {
      tf2::Transform map_to_merge;
      tf2::fromMsg(map_poses[i], map_to_merge);
      tf2::Transform grid_to_map;
      tf2::fromMsg(g->info.origin, grid_to_map);
      valid.emplace_back(g.get(), map_to_merge * grid_to_map);
    }
  }
  if (valid.empty()) {
    return false;
  }

  const double res = static_cast<double>(valid[0].first->info.resolution);
  for (const auto& v : valid) {
    if (std::abs(static_cast<double>(v.first->info.resolution) - res) > 1e-5) {
      RCLCPP_WARN_THROTTLE(
        logger_, *this->get_clock(), 5000,
        "expand_slam_maps_to_common_canvas: resolutions differ; cannot merge.");
      return false;
    }
  }

  const double margin = expand_slam_maps_margin_m_;
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  for (const auto& [map, grid_to_merge] : valid) {
    const double w = static_cast<double>(map->info.width) * res;
    const double h = static_cast<double>(map->info.height) * res;
    /* Corners of the grid in the merge frame; same idea as gingineer bottom_* offsets. */
    for (const auto& c : {tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(w, 0.0, 0.0),
                          tf2::Vector3(0.0, h, 0.0), tf2::Vector3(w, h, 0.0)}) {
      const tf2::Vector3 p = grid_to_merge * c;
      min_x = std::min(min_x, p.x());
      min_y = std::min(min_y, p.y());
      max_x = std::max(max_x, p.x());
      max_y = std::max(max_y, p.y());
    }
  }

  min_x -= margin;
  min_y -= margin;
  max_x += margin;
  max_y += margin;

  const unsigned out_w = static_cast<unsigned>(std::ceil((max_x - min_x) / res + 1e-9));
  const unsigned out_h = static_cast<unsigned>(std::ceil((max_y - min_y) / res + 1e-9));
  if (out_w == 0U || out_h == 0U || out_w > 20000U || out_h > 20000U) {
    RCLCPP_WARN(
      logger_, "expand_slam_maps_to_common_canvas: invalid canvas size %u x %u", out_w, out_h);
    return false;
  }

  for (const auto& [map, grid_to_merge] : valid) {
    auto expanded = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    expanded->header = map->header;
    expanded->info.map_load_time = map->info.map_load_time;
    expanded->info.resolution = static_cast<float>(res);
    expanded->info.width = out_w;
    expanded->info.height = out_h;
    expanded->info.origin.position.x = min_x;
    expanded->info.origin.position.y = min_y;
    expanded->info.origin.position.z = map->info.origin.position.z;
    expanded->info.origin.orientation.w = 1.0;
    expanded->data.assign(static_cast<size_t>(out_w) * out_h, static_cast<int8_t>(-1));

    /* Look up every canvas cell centre in the source grid (inverse mapping), so a
     * rotated init_pose leaves no holes. */
    const tf2::Transform merge_to_grid = grid_to_merge.inverse();
    const int w = static_cast<int>(map->info.width);
    const int h = static_cast<int>(map->info.height);
    for (unsigned oy = 0; oy < out_h; ++oy) {
      for (unsigned ox = 0; ox < out_w; ++ox) {
        const tf2::Vector3 p = merge_to_grid * tf2::Vector3(min_x + (ox + 0.5) * res,
                                                            min_y + (oy + 0.5) * res, 0.0);
        const int ix = static_cast<int>(std::floor(p.x() / res));
        const int iy = static_cast<int>(std::floor(p.y() / res));
        if (ix < 0 || iy < 0 || ix >= w || iy >= h) {
          continue;
        }
        expanded->data[static_cast<size_t>(oy) * out_w + ox] =
          map->data[static_cast<size_t>(iy) * w + static_cast<size_t>(ix)];
      }
    }
    out.push_back(std::move(expanded));
  }

  return true;
}

void MapMerge::publishRobotMapTf(MapSubscription& subscription)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->now();
  t.header.frame_id = world_frame_;
  t.child_frame_id = robot_map_frame_;
  // same placement as in the merged map
  t.transform = expand_slam_maps_apply_init_pose_ ? subscription.initial_pose : identityTransform();
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back(t);

  const std::string topic = ros1_names::append(subscription.robot_name, "tf_static");
  // latched like tf2_ros::StaticTransformBroadcaster
  subscription.tf_static_pub = this->create_publisher<tf2_msgs::msg::TFMessage>(
      topic, rclcpp::QoS(1).transient_local().reliable());
  subscription.tf_static_pub->publish(msg);
  RCLCPP_INFO(logger_, "Publishing %s -> %s on %s", world_frame_.c_str(),
              robot_map_frame_.c_str(), topic.c_str());
}

void MapMerge::poseEstimation()
{
  RCLCPP_DEBUG(logger_, "Grid pose estimation started.");
  RCLCPP_INFO_ONCE(logger_, "Grid pose estimation started.");
  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr> grids;
  grids.reserve(subscriptions_size_);

  {
    // We don't lock since because of ROS2 default executor only a callback can run
    // boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);
    for (auto& subscription : subscriptions_) {
      // std::lock_guard<std::mutex> s_lock(subscription.mutex);
      grids.push_back(subscription.readonly_map);
    }
  }

  // Print grids size
  // RCLCPP_INFO(logger_, "Grids size: %d", grids.size());

  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  pipeline_.feed(grids.begin(), grids.end());
  // TODO allow user to change feature type
  bool success = pipeline_.estimateTransforms(logger_,combine_grids::FeatureType::AKAZE,
                               confidence_threshold_);
  // bool success = pipeline_.estimateTransforms(logger_, combine_grids::FeatureType::SURF,
  //                              confidence_threshold_);
  // bool success = pipeline_.estimateTransforms(logger_, combine_grids::FeatureType::ORB,
  //                              confidence_threshold_);
  if (!success) {
    RCLCPP_INFO(logger_, "No grid poses estimated");
  }
}

// void MapMerge::fullMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& msg,
//                              MapSubscription& subscription)
void MapMerge::fullMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
                     MapSubscription& subscription)
{
  RCLCPP_DEBUG(logger_, "received full map update");
  // RCLCPP_INFO(logger_, "received full map update");
  if (msg->header.frame_id.empty() || msg->header.frame_id == "map") {
    msg->header.frame_id = ros1_names::append(subscription.robot_name, "map");
  }
  std::lock_guard<std::mutex> lock(subscription.mutex);
  if (subscription.readonly_map){
    // ros2 header .stamp don't support > operator, we need to create them explicitly
    auto t1 = rclcpp::Time(subscription.readonly_map->header.stamp);
    auto t2 = rclcpp::Time(msg->header.stamp);
    if (t1 > t2) {
      // we have been overrunned by faster update. our work was useless.
      return;
    }
  }

  subscription.readonly_map = msg;
  subscription.writable_map = nullptr;
}

// void MapMerge::partialMapUpdate(
//     const map_msgs::OccupancyGridUpdate::ConstPtr& msg,
//     MapSubscription& subscription)
void MapMerge::partialMapUpdate(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg,
                        MapSubscription& subscription)
{
  RCLCPP_DEBUG(logger_, "received partial map update");

  if (msg->x < 0 || msg->y < 0) {
    RCLCPP_ERROR(logger_, "negative coordinates, invalid update. x: %d, y: %d", msg->x,
              msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr readonly_map;  // local copy
  {
    // load maps
    std::lock_guard<std::mutex> lock(subscription.mutex);
    map = subscription.writable_map;
    readonly_map = subscription.readonly_map;
  }

  if (!readonly_map) {
    RCLCPP_WARN(logger_, "received partial map update, but don't have any full map to "
             "update. skipping.");
    return;
  }

  // // we don't have partial map to take update, we must copy readonly map and
  // // update new writable map
  if (!map) {
    map.reset(new nav_msgs::msg::OccupancyGrid(*readonly_map));
  }

  size_t grid_xn = map->info.width;
  size_t grid_yn = map->info.height;

  if (xn > grid_xn || x0 > grid_xn || yn > grid_yn || y0 > grid_yn) {
    RCLCPP_WARN(logger_, "received update doesn't fully fit into existing map, "
             "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
             "map is: [0, %lu], [0, %lu]",
             x0, xn, y0, yn, grid_xn, grid_yn);
  }

  // update map with data
  size_t i = 0;
  for (size_t y = y0; y < yn && y < grid_yn; ++y) {
    for (size_t x = x0; x < xn && x < grid_xn; ++x) {
      size_t idx = y * grid_xn + x;  // index to grid for this specified cell
      map->data[idx] = msg->data[i];
      ++i;
    }
  }
  // update time stamp
  map->header.stamp = msg->header.stamp;

  {
    // store back updated map
    std::lock_guard<std::mutex> lock(subscription.mutex);
    if (subscription.readonly_map){
      // ros2 header .stamp don't support > operator, we need to create them explicitly
      auto t1 = rclcpp::Time(subscription.readonly_map->header.stamp);
      auto t2 = rclcpp::Time(map->header.stamp);
      if (t1 > t2) {
        // we have been overrunned by faster update. our work was useless.
        return;
      }
    }
    subscription.writable_map = map;
    subscription.readonly_map = map;
  }
}

std::string MapMerge::robotNameFromTopic(const std::string& topic)
{
  return ros1_names::parentNamespace(topic);
}

/* identifies topic via suffix */
bool MapMerge::isRobotMapTopic(const std::string topic, std::string type)
{
  /* test whether topic is robot_map_topic_ */
  std::string topic_namespace = ros1_names::parentNamespace(topic);
  bool is_map_topic = ros1_names::append(topic_namespace, robot_map_topic_) == topic;

  // /* test whether topic contains *anywhere* robot namespace */
  auto pos = topic.find(robot_namespace_);
  bool contains_robot_namespace = pos != std::string::npos;

  // /* we support only occupancy grids as maps */
  bool is_occupancy_grid = type == "nav_msgs/msg/OccupancyGrid";

  // /* we don't want to subscribe on published merged map */
  bool is_our_topic = merged_map_publisher_->get_topic_name() == topic;

  return is_occupancy_grid && !is_our_topic && contains_robot_namespace &&
         is_map_topic;
}

/*
 * Get robot's initial position
 */
bool MapMerge::getInitPose(const std::string& name,
                           geometry_msgs::msg::Transform& pose)
{
  std::string merging_namespace = ros1_names::append(name, "map_merge");
  double yaw = 0.0;

  bool success =
      this->get_parameter(ros1_names::append(merging_namespace, "init_pose_x"),
                      pose.translation.x) &&
      this->get_parameter(ros1_names::append(merging_namespace, "init_pose_y"),
                      pose.translation.y) &&
      this->get_parameter(ros1_names::append(merging_namespace, "init_pose_z"),
                      pose.translation.z) &&
      this->get_parameter(ros1_names::append(merging_namespace, "init_pose_yaw"),
                      yaw);

  tf2::Quaternion q;
  q.setEuler(0., 0., yaw);
  pose.rotation = toMsg(q);

  return success;
}
}  // namespace map_merge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  // // this package is still in development -- start with debugging enabled
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                    ros::console::levels::Debug)) {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  // ROS2 code
  auto node = std::make_shared<map_merge::MapMerge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
