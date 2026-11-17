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

#ifndef MAP_MERGE_H_
#define MAP_MERGE_H_

#include <atomic>
#include <forward_list>
#include <mutex>
#include <unordered_map>

#include <combine_grids/merging_pipeline.h>
#include <geometry_msgs/msg/transform.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <boost/thread.hpp>

namespace map_merge
{
struct MapSubscription {
  // protects consistency of writable_map and readonly_map
  // also protects reads and writes of shared_ptrs
  std::mutex mutex;

  /** e.g. "robot1" — used for stable merge order (forward_list iterates newest-first). */
  std::string robot_name;
  geometry_msgs::msg::Transform initial_pose;
  nav_msgs::msg::OccupancyGrid::SharedPtr writable_map;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr readonly_map;

  // ros::Subscriber map_sub;
  // ros::Subscriber map_updates_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_updates_sub;
  // world_frame -> robot map frame, latched on the robot's own TF tree
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub;
  // merged map with this robot's own map on top (publish_robot_maps)
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr robot_merged_map_pub;
};

class MapMerge : public rclcpp::Node
{
private:
  /* parameters */
  double merging_rate_;
  double discovery_rate_;
  double estimation_rate_;
  double confidence_threshold_;
  std::string robot_map_topic_;
  std::string robot_map_updates_topic_;
  std::string robot_namespace_;
  std::string world_frame_;
  bool have_initial_poses_;
  /** Like gingineer map_expansion: embed each SLAM map in one shared canvas using origins only (no TF between maps). */
  bool expand_slam_maps_to_common_canvas_;
  double expand_slam_maps_margin_m_;
  /** If true (with expansion on), place each map by its init_pose_* (pose of the robot's map frame relative to the
   * first robot's). If false, all map frames are assumed to coincide. */
  bool expand_slam_maps_apply_init_pose_;
  /** Publish world_frame -> robot_map_frame on each robot's <robot>/tf_static (expansion with known poses
   * only), so robots can use the merged map, e.g. for exploring global frontiers. */
  bool publish_robot_map_tf_;
  std::string robot_map_frame_;
  /** Also publish <robot>/<robot_merged_map_topic>: the merged map where each robot's own map wins wherever
   * it knows a cell. Other robots map a robot as an obstacle; planning on the plain merged map, it would
   * stand inside its own obstacle. */
  bool publish_robot_maps_;
  std::string robot_merged_map_topic_;

  // publishing
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_map_publisher_;
  // maps robots namespaces to maps. does not own
  std::unordered_map<std::string, MapSubscription*> robots_;
  // owns maps -- iterator safe
  std::forward_list<MapSubscription> subscriptions_;
  size_t subscriptions_size_;
  boost::shared_mutex subscriptions_mutex_;
  combine_grids::MergingPipeline pipeline_;
  std::mutex pipeline_mutex_;

  rclcpp::Logger logger_;

  // timers
  rclcpp::TimerBase::SharedPtr map_merging_timer_;
  rclcpp::TimerBase::SharedPtr topic_subscribing_timer_;
  rclcpp::TimerBase::SharedPtr pose_estimation_timer_;

  std::string robotNameFromTopic(const std::string& topic);
  // bool isRobotMapTopic(const ros::master::TopicInfo& topic);
  bool isRobotMapTopic(const std::string topic, std::string type);
  bool getInitPose(const std::string& name, geometry_msgs::msg::Transform& pose);
  void publishRobotMapTf(MapSubscription& subscription);
  void publishRobotMaps(const nav_msgs::msg::OccupancyGrid& merged,
                        const std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr>& own_maps,
                        const std::vector<MapSubscription*>& robots);

  void fullMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
                     MapSubscription& map);
  void partialMapUpdate(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg,
                        MapSubscription& map);

  /** slam_toolbox: expand each map to identical width/height/origin (issue #10 / gingineer).
   * map_poses[i] is the pose of in[i]'s map frame in the merge frame (init_pose_* or identity). */
  bool expandSlamMapsToCommonCanvas(const std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr>& in,
                                  const std::vector<geometry_msgs::msg::Transform>& map_poses,
                                  std::vector<nav_msgs::msg::OccupancyGrid::ConstSharedPtr>& out);

public:
  MapMerge();

  void topicSubscribing();
  void mapMerging();
  /**
   * @brief Estimates initial positions of grids
   * @details Relevant only if initial poses are not known
   */
  void poseEstimation();
};

}  // namespace map_merge

#endif /* MAP_MERGE_H_ */
