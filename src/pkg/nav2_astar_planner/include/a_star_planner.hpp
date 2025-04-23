#ifndef NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

namespace nav2_astar_planner
{

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  struct Node {
    int x, y; // Node coordinates in the gridmap
    double g, h; // Costs: g (distance from start), h (heuristic)
    Node* parent;  // Parent node for path reconstruction
    Node(int x, int y, double g, double h, Node* parent = nullptr)
      : x(x), y(y), g(g), h(h), parent(parent) {}   // Construction of the node
    double f() const { return g + h; }   // A* evaluation function (just return g + h)
  };

  // Comparator for the priority queue (min-heap based on f value)
  //  Note: priority queue automatically order object bas
  struct CompareNodes {
    bool operator()(const Node* a, const Node* b) const {
      return a->f() > b->f();
    }
  };

  std::vector<std::pair<int, int>> get_neighbors(int x, int y);
  double heuristic(int x1, int y1, int x2, int y2);

  nav2_util::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string global_frame_;
};

}  // namespace nav2_astar_planner

#endif  // NAV2_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
