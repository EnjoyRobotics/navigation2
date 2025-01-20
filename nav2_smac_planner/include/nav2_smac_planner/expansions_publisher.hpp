#ifndef NAV2_SMAC_PLANNER__EXPANSIONS_PUBLISHER_HPP_
#define NAV2_SMAC_PLANNER__EXPANSIONS_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "nav2_smac_planner/types.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_smac_planner
{

template <typename NodeT>
class ExpansionsPublisher
{
public:
  using Coordinates = typename NodeT::Coordinates;
  using ExpansionT = nav2_smac_planner::ExpansionT<Coordinates>;

  ExpansionsPublisher(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    std::string base_topic_name)
  : node_(node),
    costmap_ros_(costmap_ros),
    base_topic_name_(base_topic_name),
    logger_(rclcpp::get_logger(base_topic_name_ + " ExpansionsPublisher"))
  {}

  void publish(const ExpansionT & expansions)
  {
    // First get max costs to normalize
    std::unordered_map<std::string, float> max_costs;
    for (const auto & [cost_name, cost_list] : expansions) {
      float & max_cost = max_costs[cost_name];
      max_cost = 0.0;

      for (const auto & [index, coord_and_cost] : cost_list) {
        const float & cost = coord_and_cost.second;
        max_cost = std::max(max_cost, cost);
      }

      RCLCPP_DEBUG_STREAM(logger_, "Max cost for " << cost_name << ": " << max_cost);
    }

    // Normalize odom and traversal together, the rest only compared to themselves
    float max_odom_trav = std::max(max_costs["odom"], max_costs["trav"]);

    for (const auto & [cost_name, cost_list] : expansions) {
      // Create publisher if it doesn't exist
      if (publishers_.find(cost_name) == publishers_.end()) {
        publishers_[cost_name] = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
          base_topic_name_ + "/" + cost_name, 10);
        publishers_[cost_name]->on_activate();
      }

      nav_msgs::msg::OccupancyGrid occupancy_grid;
      occupancy_grid.header.frame_id = costmap_ros_->getGlobalFrameID();
      occupancy_grid.header.stamp = node_->now();
      occupancy_grid.info.resolution = costmap_ros_->getCostmap()->getResolution();
      occupancy_grid.info.width = costmap_ros_->getCostmap()->getSizeInCellsX();
      occupancy_grid.info.height = costmap_ros_->getCostmap()->getSizeInCellsY();
      occupancy_grid.info.origin.position.x = costmap_ros_->getCostmap()->getOriginX();
      occupancy_grid.info.origin.position.y = costmap_ros_->getCostmap()->getOriginY();
      occupancy_grid.info.origin.orientation.w = 1.0;

      occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height, 0);

      // Set value
      for (const auto & [index, coord_and_cost] : cost_list) {
        const Coordinates & coords = coord_and_cost.first;
        const float & cost = coord_and_cost.second;

        float max_cost = max_costs[cost_name];
        if (cost_name == "odom" || cost_name == "trav") {
          max_cost = max_odom_trav;
        }
        if (max_cost <= 0.0) {
          max_cost = 1.0;
        }

        char value = static_cast<char>(cost * 98.0 / max_cost);
        uint index_1d = costmap_ros_->getCostmap()->getIndex(coords.x, coords.y);
        occupancy_grid.data[index_1d] = value;
      }

      publishers_[cost_name]->publish(occupancy_grid);
    }
  }

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unordered_map<std::string, rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> publishers_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string base_topic_name_;
  rclcpp::Logger logger_;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__EXPANSIONS_PUBLISHER_HPP_
