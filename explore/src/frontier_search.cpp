#include <explore/costmap_tools.h>
#include <explore/frontier_search.h>

#include <geometry_msgs/msg/point.hpp>
#include <mutex>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_exploration
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size, rclcpp::Logger logger,
                               double min_unknown_area)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
  , min_unknown_area_(min_unknown_area)
  , logger_(logger)
{
}

std::vector<Frontier>
FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    RCLCPP_ERROR(logger_, "[FrontierSearch] Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    RCLCPP_WARN(logger_, "[FrontierSearch] Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() >=
                min_frontier_size_ &&
            !isEnclosedPocket(new_frontier)) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // Unknown cells within 2 cells of an obstacle are almost always the never-seen
  // inside of a wall or box, not an opening: LiDAR noise marks single face cells
  // free, so the unknown cells behind them must not count either (2 cells = 0.1 m
  // at 5 cm resolution, the arena walls' thickness)
  constexpr int obstacle_clearance = 2;
  unsigned int mx, my;
  costmap_->indexToCells(idx, mx, my);
  for (int dy = -obstacle_clearance; dy <= obstacle_clearance; ++dy) {
    for (int dx = -obstacle_clearance; dx <= obstacle_clearance; ++dx) {
      const int x = static_cast<int>(mx) + dx;
      const int y = static_cast<int>(my) + dy;
      if (x >= 0 && y >= 0 && x < static_cast<int>(size_x_) && y < static_cast<int>(size_y_) &&
          map_[costmap_->getIndex(x, y)] == LETHAL_OBSTACLE) {
        return false;
      }
    }
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

bool FrontierSearch::isEnclosedPocket(const Frontier& frontier) const
{
  if (min_unknown_area_ <= 0.0) {
    return false;
  }
  // Flood-fill the unknown cells connected to the frontier. Stop as soon as the
  // area is large enough (open, worth exploring); running out of cells first means
  // the unknown space is enclosed by known cells and small.
  const double resolution = costmap_->getResolution();
  const size_t max_cells =
      static_cast<size_t>(min_unknown_area_ / (resolution * resolution));
  std::vector<bool> seen(size_x_ * size_y_, false);
  std::queue<unsigned int> fill;
  for (const auto& p : frontier.points) {
    unsigned int mx, my;
    if (costmap_->worldToMap(p.x, p.y, mx, my)) {
      const unsigned int idx = costmap_->getIndex(mx, my);
      if (!seen[idx]) {
        seen[idx] = true;
        fill.push(idx);
      }
    }
  }
  size_t cells = 0;
  while (!fill.empty()) {
    const unsigned int idx = fill.front();
    fill.pop();
    if (++cells >= max_cells) {
      return false;
    }
    for (unsigned int nbr : nhood4(idx, *costmap_)) {
      if (!seen[nbr] && map_[nbr] == NO_INFORMATION) {
        seen[nbr] = true;
        fill.push(nbr);
      }
    }
  }
  return true;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  // min_distance is already in metres, size in cells: scale only the size, or
  // the distance counts 1/resolution (20x at 5 cm) less than intended
  return (potential_scale_ * frontier.min_distance) -
         (gain_scale_ * frontier.size * costmap_->getResolution());
}
}  // namespace frontier_exploration
