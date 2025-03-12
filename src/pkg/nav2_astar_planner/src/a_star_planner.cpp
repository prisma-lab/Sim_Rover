#include "a_star_planner.hpp"

namespace nav2_astar_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
}

void AStarPlanner::cleanup() {}
void AStarPlanner::activate() {}
void AStarPlanner::deactivate() {}

std::vector<std::pair<int, int>> AStarPlanner::get_neighbors(int x, int y)
{
  std::vector<std::pair<int, int>> neighbors;
  std::vector<std::pair<int, int>> deltas = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
  for (auto [dx, dy] : deltas) {
    int nx = x + dx, ny = y + dy;
    if (nx >= 0 && ny >= 0 && nx < costmap_->getSizeInCellsX() && ny < costmap_->getSizeInCellsY()) {
      if (costmap_->getCost(nx, ny) < nav2_costmap_2d::LETHAL_OBSTACLE) {
        neighbors.emplace_back(nx, ny);
      }
    }
  }
  return neighbors;
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2)
{
  return std::hypot(x1 - x2, y1 - y2);
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  
  unsigned int start_x, start_y, goal_x, goal_y;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
  
  std::priority_queue<Node*, std::vector<Node*>, CompareNodes> open_set;
  std::unordered_map<int, Node*> all_nodes;
  Node* start_node = new Node(start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y));
  open_set.push(start_node);
  all_nodes[start_x * costmap_->getSizeInCellsY() + start_y] = start_node;

  while (!open_set.empty()) {
    Node* current = open_set.top();
    open_set.pop();
    
    if (current->x == goal_x && current->y == goal_y) {
      while (current) {
        geometry_msgs::msg::PoseStamped pose;
        double wx, wy;
        costmap_->mapToWorld(current->x, current->y, wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.orientation.w = 1.0;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        global_path.poses.push_back(pose);
        current = current->parent;
      }
      std::reverse(global_path.poses.begin(), global_path.poses.end());
      return global_path;
    }
    
    for (auto [nx, ny] : get_neighbors(current->x, current->y)) {
      double new_g = current->g + 1;
      int index = nx * costmap_->getSizeInCellsY() + ny;
      if (!all_nodes.count(index) || new_g < all_nodes[index]->g) {
        Node* new_node = new Node(nx, ny, new_g, heuristic(nx, ny, goal_x, goal_y), current);
        open_set.push(new_node);
        all_nodes[index] = new_node;
      }
    }
  }
  return global_path;
}

}  // namespace nav2_astar_planner

PLUGINLIB_EXPORT_CLASS(nav2_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)
