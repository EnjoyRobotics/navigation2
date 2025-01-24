// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2020, Applied Electric Vehicles Pty Ltd
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

#include <unordered_map>
#include <omp.h>
#include <cmath>
#include <stdexcept>
#include <memory>
#include <algorithm>
#include <limits>
#include <type_traits>
#include <chrono>
#include <thread>
#include <utility>
#include <vector>

#include "nav2_smac_planner/a_star.hpp"
#include "nav_msgs/msg/path.hpp"
#include "angles/angles.h"

using namespace std::chrono;  // NOLINT

namespace nav2_smac_planner
{

template<typename NodeT>
AStarAlgorithm<NodeT>::AStarAlgorithm(
  const MotionModel & motion_model,
  const SearchInfo & search_info)
: _traverse_unknown(true),
  _is_initialized(false),
  _max_iterations(0),
  _max_planning_time(0),
  _x_size(0),
  _y_size(0),
  _search_info(search_info),
  _goal_coordinates(Coordinates()),
  _start(nullptr),
  _goal(nullptr),
  _motion_model(motion_model)
{
  _graph.reserve(100000);
}

template<typename NodeT>
AStarAlgorithm<NodeT>::~AStarAlgorithm()
{
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::initialize(
  const bool & allow_unknown,
  int & max_iterations,
  const int & max_on_approach_iterations,
  const double & max_planning_time,
  const float & lookup_table_size,
  const unsigned int & dim_3_size)
{
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _max_on_approach_iterations = max_on_approach_iterations;
  _max_planning_time = max_planning_time;
  if(!_is_initialized) {
    NodeT::precomputeDistanceHeuristic(lookup_table_size, _motion_model, dim_3_size, _search_info);
  }
  _is_initialized = true;
  _dim3_size = dim_3_size;
  _expander = std::make_unique<AnalyticExpansion<NodeT>>(
    _motion_model, _search_info, _traverse_unknown, _dim3_size);
}

template<>
void AStarAlgorithm<Node2D>::initialize(
  const bool & allow_unknown,
  int & max_iterations,
  const int & max_on_approach_iterations,
  const double & max_planning_time,
  const float & /*lookup_table_size*/,
  const unsigned int & dim_3_size)
{
  _traverse_unknown = allow_unknown;
  _max_iterations = max_iterations;
  _max_on_approach_iterations = max_on_approach_iterations;
  _max_planning_time = max_planning_time;

  if (dim_3_size != 1) {
    throw std::runtime_error("Node type Node2D cannot be given non-1 dim 3 quantization.");
  }
  _dim3_size = dim_3_size;
  _expander = std::make_unique<AnalyticExpansion<Node2D>>(
    _motion_model, _search_info, _traverse_unknown, _dim3_size);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setCollisionChecker(GridCollisionChecker * collision_checker)
{
  _collision_checker = collision_checker;
  _costmap = collision_checker->getCostmap();
  unsigned int x_size = _costmap->getSizeInCellsX();
  unsigned int y_size = _costmap->getSizeInCellsY();

  clearGraph();

  if (getSizeX() != x_size || getSizeY() != y_size) {
    _x_size = x_size;
    _y_size = y_size;
    NodeT::initMotionModel(_motion_model, _x_size, _y_size, _dim3_size, _search_info);
  }
  _expander->setCollisionChecker(collision_checker);
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr AStarAlgorithm<NodeT>::addToGraph(
  const unsigned int & index)
{
  auto iter = _graph.find(index);
  if (iter != _graph.end()) {
    return &(iter->second);
  }

  return &(_graph.emplace(index, NodeT(index)).first->second);
}

template<>
void AStarAlgorithm<Node2D>::setStart(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  if (dim_3 != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero starting dim 3.");
  }
  _start = addToGraph(Node2D::getIndex(mx, my, getSizeX()));
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setStart(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  _start = addToGraph(NodeT::getIndex(mx, my, dim_3));
  _start->setPose(
    Coordinates(
      static_cast<float>(mx),
      static_cast<float>(my),
      static_cast<float>(dim_3)));
}

template<>
void AStarAlgorithm<Node2D>::setGoal(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  if (dim_3 != 0) {
    throw std::runtime_error("Node type Node2D cannot be given non-zero goal dim 3.");
  }

  _goal = addToGraph(Node2D::getIndex(mx, my, getSizeX()));
  _goal_coordinates = Node2D::Coordinates(mx, my);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::setGoal(
  const unsigned int & mx,
  const unsigned int & my,
  const unsigned int & dim_3)
{
  _goal = addToGraph(NodeT::getIndex(mx, my, dim_3));

  typename NodeT::Coordinates goal_coords(
    static_cast<float>(mx),
    static_cast<float>(my),
    static_cast<float>(dim_3));

  if (!_search_info.cache_obstacle_heuristic || goal_coords != _goal_coordinates) {
    if (!_start) {
      throw std::runtime_error("Start must be set before goal.");
    }

    NodeT::resetObstacleHeuristic(_costmap, _start->pose.x, _start->pose.y, mx, my);
  }

  _goal_coordinates = goal_coords;
  _goal->setPose(_goal_coordinates);
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::areInputsValid()
{
  // Check if graph was filled in
  if (_graph.empty()) {
    throw std::runtime_error("Failed to compute path, no costmap given.");
  }

  // Check if points were filled in
  if (!_start || !_goal) {
    throw std::runtime_error("Failed to compute path, no valid start or goal given.");
  }

  // Check if ending point is valid
  if (getToleranceHeuristic() < 0.001 &&
    !_goal->isNodeValid(_traverse_unknown, _collision_checker))
  {
    throw nav2_core::GoalOccupied("Goal was in lethal cost");
  }

  // Check if starting point is valid
  if (!_start->isNodeValid(_traverse_unknown, _collision_checker)) {
    throw nav2_core::StartOccupied("Start was in lethal cost");
  }

  return true;
}

template<typename NodeT>
RolloutT AStarAlgorithm<NodeT>::getRollout() const
{
  RolloutT rollout;

  // TODO should these be parameters?
  static const size_t n_points = 6;  // for rollout
  static const float min_time_step = 0.2;  // for rollout

  // Calculate rollout
  if (!odometry) {
    // RCLCPP_ERROR(_logger, "No odometry received for rollout");
    return {};
  }

  const float vx_abs = std::abs(odometry->twist.twist.linear.x);
  if (_search_info.odom_min_vel > 0.0 && vx_abs < _search_info.odom_min_vel) {
    // RCLCPP_ERROR(_logger, "Odometry velocity below minimum for rollout");
    return {};
  }

  if (_search_info.odom_penalty <= 0.0 || _search_info.odom_rollout_time <= 0.0) {
    // RCLCPP_ERROR(_logger, "Rollout disabled");
    return {};
  }

  geometry_msgs::msg::Pose2D start_pose;
  start_pose.x = odometry->pose.pose.position.x;
  start_pose.y = odometry->pose.pose.position.y;
  start_pose.theta = tf2::getYaw(odometry->pose.pose.orientation);

  const float time_step = std::max<float>(
    _search_info.odom_rollout_time / n_points, min_time_step);

  // TODO implement for non differential drive
  geometry_msgs::msg::Pose2D pose = start_pose;
  const float dx = odometry->twist.twist.linear.x * time_step;
  const float dtheta = odometry->twist.twist.angular.z * time_step;

  for (size_t i = 0; i < n_points; ++i) {
    if (odometry->twist.twist.angular.z == 0.0) {
      pose.x = start_pose.x + dx * i * std::cos(start_pose.theta);
      pose.y = start_pose.y + dx * i * std::sin(start_pose.theta);
      pose.theta = start_pose.theta;
    } else {
      pose.x += dx * std::cos(pose.theta);
      pose.y += dx * std::sin(pose.theta);
      pose.theta += dtheta;
    }

    const float dist = std::hypot(
      pose.x - odometry->pose.pose.position.x,
      pose.y - odometry->pose.pose.position.y);
    const float angle = std::atan2(
      pose.y - odometry->pose.pose.position.y,
      pose.x - odometry->pose.pose.position.x);
    rollout.insert({dist, angle});
  }

  return rollout;
}

template<typename NodeT>
float AStarAlgorithm<NodeT>::getOdomCost(
  const NodePtr & node, const RolloutT & rollout)
{
  // Note: this assumes x and y node coords refer to the same pixel in the costmap
  const Coordinates node_coords =
    NodeT::getCoords(node->getIndex(), getSizeX(), getSizeDim3());
  double pixel_x, pixel_y;
  _costmap->mapToWorld(node_coords.x, node_coords.y, pixel_x, pixel_y);

  const float dist_from_center = std::hypot(
    pixel_x - odometry->pose.pose.position.x,
    pixel_y - odometry->pose.pose.position.y);

  const auto angle_iter = rollout.lower_bound(dist_from_center);
  if (angle_iter == rollout.end()) {
    return 0.0;
  }

  const float rollout_angle = angle_iter->second;
  const float pixel_angle = std::atan2(
    pixel_y - odometry->pose.pose.position.y,
    pixel_x - odometry->pose.pose.position.x);
  const float angle_diff = angles::shortest_angular_distance(pixel_angle, rollout_angle);

  return _search_info.odom_penalty * std::abs(angle_diff) / M_PI;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::createPath(
  CoordinateVector & path, int & iterations,
  const float & tolerance,
  std::shared_ptr<ExpansionT<Coordinates>> expansions)
{
  steady_clock::time_point start_time = steady_clock::now();
  _tolerance = tolerance;
  _best_heuristic_node = {std::numeric_limits<float>::max(), 0};
  clearQueue();

  if (!areInputsValid()) {
    return false;
  }

  if (expansions) {
    expansions->clear();
  } else {
    // RCLCPP_WARN(_logger, "No expansions given, not recording");
  }

  RolloutT rollout = getRollout();

  // 0) Add starting point to the open set
  addNode(0.0, getStart());
  getStart()->setAccumulatedCost(0.0);

  // Optimization: preallocate all variables
  NodePtr current_node = nullptr;
  NodePtr neighbor = nullptr;
  NodePtr expansion_result = nullptr;
  float g_cost = 0.0;
  NodeVector neighbors;
  int approach_iterations = 0;
  NeighborIterator neighbor_iterator;
  int analytic_iterations = 0;
  int closest_distance = std::numeric_limits<int>::max();

  // Given an index, return a node ptr reference if its collision-free and valid
  const unsigned int max_index = getSizeX() * getSizeY() * getSizeDim3();
  NodeGetter neighborGetter =
    [&, this](const unsigned int & index, NodePtr & neighbor_rtn) -> bool
    {
      if (index >= max_index) {
        return false;
      }

      neighbor_rtn = addToGraph(index);
      return true;
    };

  while (iterations < getMaxIterations() && !_queue.empty()) {
    // Check for planning timeout only on every Nth iteration
    if (iterations % _timing_interval == 0) {
      std::chrono::duration<double> planning_duration =
        std::chrono::duration_cast<std::chrono::duration<double>>(steady_clock::now() - start_time);
      if (static_cast<double>(planning_duration.count()) >= _max_planning_time) {
        return false;
      }
    }

    // 1) Pick Nbest from O s.t. min(f(Nbest)), remove from queue
    current_node = getNextNode();

    // We allow for nodes to be queued multiple times in case
    // shorter paths result in it, but we can visit only once
    if (current_node->wasVisited()) {
      continue;
    }

    iterations++;

    // 2) Mark Nbest as visited
    current_node->visited();

    // 2.1) Use an analytic expansion (if available) to generate a path
    expansion_result = nullptr;
    expansion_result = _expander->tryAnalyticExpansion(
      current_node, getGoal(), neighborGetter, analytic_iterations, closest_distance);
    if (expansion_result != nullptr) {
      current_node = expansion_result;
    }

    // 3) Check if we're at the goal, backtrace if required
    if (isGoal(current_node)) {
      return current_node->backtracePath(path);
    } else if (_best_heuristic_node.first < getToleranceHeuristic()) {
      // Optimization: Let us find when in tolerance and refine within reason
      approach_iterations++;
      if (approach_iterations >= getOnApproachMaxIterations()) {
        return _graph.at(_best_heuristic_node.second).backtracePath(path);
      }
    }

    // 4) Expand neighbors of Nbest not visited
    neighbors.clear();
    current_node->getNeighbors(neighborGetter, _collision_checker, _traverse_unknown, neighbors);

    for (neighbor_iterator = neighbors.begin();
      neighbor_iterator != neighbors.end(); ++neighbor_iterator)
    {
      neighbor = *neighbor_iterator;

      // 4.1) Compute the cost to go to this node
      const float accumulated_cost = current_node->getAccumulatedCost();
      const float traversal_cost = current_node->getTraversalCost(neighbor);
      const float odom_cost = rollout.empty() ? 0.0 : getOdomCost(neighbor, rollout);
      g_cost = accumulated_cost + traversal_cost + odom_cost;

      if (expansions) {
        const float heuristic_cost = getHeuristicCost(neighbor);
        const uint index = neighbor->getIndex();
        const auto coords = NodeT::getCoords(neighbor->getIndex(), getSizeX(), getSizeDim3());
        (*expansions)["odom"][index] = std::make_pair(coords, odom_cost);
        (*expansions)["trav"][index] = std::make_pair(coords, traversal_cost);
        (*expansions)["heur"][index] = std::make_pair(coords, heuristic_cost);
        (*expansions)["total"][index] = std::make_pair(coords, g_cost + heuristic_cost);
      }

      // 4.2) If this is a lower cost than prior, we set this as the new cost and new approach
      if (g_cost < neighbor->getAccumulatedCost()) {
        neighbor->setAccumulatedCost(g_cost);
        neighbor->parent = current_node;

        // 4.3) Add to queue with heuristic cost
        const float heuristic_cost = getHeuristicCost(neighbor);
        addNode(g_cost + heuristic_cost, neighbor);
      }
    }
  }

  if (_best_heuristic_node.first < getToleranceHeuristic()) {
    // If we run out of serach options, return the path that is closest, if within tolerance.
    return _graph.at(_best_heuristic_node.second).backtracePath(path);
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("nav2_smac_planner"),
      "A* failed to find a path, closest node was %f away from goal (tolerance: %f)",
      _best_heuristic_node.first, getToleranceHeuristic());
  }

  return false;
}

template<typename NodeT>
bool AStarAlgorithm<NodeT>::isGoal(NodePtr & node)
{
  return node == getGoal();
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getStart()
{
  return _start;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr & AStarAlgorithm<NodeT>::getGoal()
{
  return _goal;
}

template<typename NodeT>
typename AStarAlgorithm<NodeT>::NodePtr AStarAlgorithm<NodeT>::getNextNode()
{
  NodeBasic<NodeT> node = _queue.top().second;
  _queue.pop();
  node.processSearchNode();
  return node.graph_node_ptr;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::addNode(const float & cost, NodePtr & node)
{
  NodeBasic<NodeT> queued_node(node->getIndex());
  queued_node.populateSearchNode(node);
  _queue.emplace(cost, queued_node);
}

template<typename NodeT>
float AStarAlgorithm<NodeT>::getHeuristicCost(const NodePtr & node)
{
  const Coordinates node_coords =
    NodeT::getCoords(node->getIndex(), getSizeX(), getSizeDim3());
  float heuristic = NodeT::getHeuristicCost(
    node_coords, _goal_coordinates, _costmap);

  if (heuristic < _best_heuristic_node.first) {
    _best_heuristic_node = {heuristic, node->getIndex()};
  }

  return heuristic;
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearQueue()
{
  NodeQueue q;
  std::swap(_queue, q);
}

template<typename NodeT>
void AStarAlgorithm<NodeT>::clearGraph()
{
  Graph g;
  std::swap(_graph, g);
  _graph.reserve(100000);
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getMaxIterations()
{
  return _max_iterations;
}

template<typename NodeT>
int & AStarAlgorithm<NodeT>::getOnApproachMaxIterations()
{
  return _max_on_approach_iterations;
}

template<typename NodeT>
float & AStarAlgorithm<NodeT>::getToleranceHeuristic()
{
  return _tolerance;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeX()
{
  return _x_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeY()
{
  return _y_size;
}

template<typename NodeT>
unsigned int & AStarAlgorithm<NodeT>::getSizeDim3()
{
  return _dim3_size;
}

// Instantiate algorithm for the supported template types
template class AStarAlgorithm<Node2D>;
template class AStarAlgorithm<NodeHybrid>;
template class AStarAlgorithm<NodeLattice>;

}  // namespace nav2_smac_planner
