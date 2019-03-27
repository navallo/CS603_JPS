// Copyright (c) 2018 joydeepb@cs.umass.edu
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "jps.h"
#include "simple_queue.h"
#include "util/timer.h"

using Eigen::Vector2i;
using std::make_pair;
using std::unordered_map;
using std::unordered_set;
using std::vector;

DEFINE_bool(nojps, false, "Disable JPS");
DEFINE_bool(nogui, false, "Disable Visualization");

namespace {
static const uint8_t kBlue[] = {0, 0, 255};
static const uint8_t kRed[] = {255, 0, 0};
static const uint8_t kGreen[] = {0, 255, 0};
static const uint8_t kYellow[] = {255, 255, 0};
static const uint8_t kGrey[] = {128, 128, 128};

}  // namespace

namespace astarplanner {

float Dist(const Node& v1, const Node& v2) {
  return ((v1 - v2).cast<float>().norm());
}

// Returns the path to goal, in reverse order.
void AStarPlanner::GetPath(const NodeMap& parent_map,
                           const Node& goal,
                           Path* path_ptr) {
  // TODO
}

void AStarPlanner::InitVisualization(const Map& map) {
  display_ = new cimg_library::CImgDisplay(viz_image_, "A* with JPS");
  viz_image_ = cimg_library::CImg<uint8_t>(
      map.width(), map.height(), 1, 3, 0);
  // Draw map
  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      if (map.Occupied(x, y)) {
        viz_image_(x, y, 0, 0) = 255;
        viz_image_(x, y, 0, 1) = 255;
        viz_image_(x, y, 0, 2) = 255;
      }
    }
  }
  acc_viz_image_ = viz_image_;
}

void AStarPlanner::Visualize(const Map& map,
                             const Node& start,
                             const Node& goal,
                             const Node& current,
                             const NodeMap& parent_map) {
  static double t_last_visualized = 0;
  static const double kMinVizInterval = 0.02;
  if (t_last_visualized > GetMonotonicTime() - kMinVizInterval) return;

  for (const auto& p : parent_map) {
    acc_viz_image_.draw_line(
      p.first.x(), p.first.y(), p.second.x(), p.second.y(), kBlue);
  }
  acc_viz_image_.draw_point(current.x(), current.y(), kYellow);

  viz_image_ = acc_viz_image_;

  viz_image_.draw_circle(current.x(), current.y(), 2, kYellow);
  viz_image_.draw_circle(start.x(), start.y(), 2, kRed);
  viz_image_.draw_circle(goal.x(), goal.y(), 2, kGreen);

  display_->display(viz_image_);
  // if (display_->is_key()) exit(0);
  t_last_visualized = GetMonotonicTime();
}

void AStarPlanner::DrawPath(const Path& path) {
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    viz_image_.draw_line(
        path[i].x(),
        path[i].y(),
        path[i + 1].x(),
        path[i + 1].y(),
        kGreen);
  }
  viz_image_.draw_circle(
      path[0].x(),
      path[0].y(),
      3,
      kGreen);
  viz_image_.draw_circle(
      path.back().x(),
      path.back().y(),
      3,
      kRed);
}

bool AStarPlanner::Plan(const Map& map,
                        const Node& start,
                        const Node& goal,
                        Path* path) {
  if (!map.ValidNode(start)) {
    printf("Invalid start location.\n");
    return false;
  }
  if (!map.ValidNode(goal)) {
    printf("Invalid goal location.\n");
    return false;
  }
  if (map.Occupied(start) || map.Occupied(goal)) {
    printf("No path possible. Start unreachable:%d goal unreachable:%d\n",
           1 - static_cast<int>(map.Occupied(start)),
           1 - static_cast<int>(map.Occupied(goal)));
    return false;
  }
  if (!FLAGS_nogui) InitVisualization(map);
  // Initialize parent map.
  parent_map_.clear();
  // Clear all G values.
  g_values_.clear();
  // Initialize an empty priority queue.
  SimpleQueue<Node, AStarPriority> queue;
  // Add start to priority queue.
  const float start_heuristic = 0; // TODO
  queue.Push(start, AStarPriority(0, start_heuristic));
  g_values_[start] = 0;
  // Clear the closed set.
  closed_set_.clear();

  // ==============================================================
  // Priority queue.
  // ==============================================================
  // Adding a node to the priority queue, with g-value and h-value:
  // queue.Push(node, AStarPriority(g, h));

  // Checking if the queue is empty:
  // if (queue.Empty()) { ... }

  // Get the current node with the highest priority (smallest f = g + h).
  // Node current_node = queue.Pop();

  // ==============================================================
  // Sets.
  // ==============================================================
  // Insert a node into the closed set:
  // closed_set_.insert(node);

  // Check if a node exists in the closed set:
  // if (closed_set_.find(node) == closed_set_.end()) {
  //   // node was not found in the closed set.
  // } else {
  //   // node was found in the closed set.
  // }

  // ==============================================================
  // Parent map and g values are stored using a hash table.
  // ==============================================================
  // If a node does not exist in the hash table, setting its parent or
  // g-value will initialize a new entry in the hash table:
  // g_values_[node] = g;
  // parent_map_[node] = parent_node;

  // If a node already exists in the hash table, you can update its value by
  // setting it to a new value, which looks identical to its creation:
  // g_values_[node] = g;
  // parent_map_[node] = parent_node;

  // Checking if a node exists in the hash table or not:
  // if (g_values_.find(node) == g_values_.end()) {
  //   // Does not exist.
  // } else {
  //   // Exists.
  // }

  // ==============================================================
  // Visualization
  // ==============================================================
  // Visualize the current state of the plan search. Useful to call for every
  // iteration, to visualize the progress of the search.
  // Visualize(map, start, goal, current, parent_map_);

  // To visualize the path found:
  GetPath(parent_map_, goal, path);
  if (!FLAGS_nogui) {
    DrawPath(*path);
    display_->display(viz_image_);
    while (!display_->is_closed() && !display_->is_key()) display_->wait();
  }

  return false;
}

}  // namespace astarplanner
