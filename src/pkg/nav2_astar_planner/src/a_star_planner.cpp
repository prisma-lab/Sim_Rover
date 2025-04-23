#include "a_star_planner.hpp"

namespace nav2_astar_planner
{

//Configure virtual method 
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

//Unused virtual methods
void AStarPlanner::cleanup() {}
void AStarPlanner::activate() {}
void AStarPlanner::deactivate() {}


// Function to get valid neighbors of a cell in the grid
//  Neighbors are returned as a list of pairs (i.e., cells in the gridmap)
std::vector<std::pair<int, int>> AStarPlanner::get_neighbors(int x, int y)
{
  std::vector<std::pair<int, int>> neighbors;

  // 4-connected grid
  std::vector<std::pair<int, int>> deltas = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
  //for each pair in deltas
  for (auto [dx, dy] : deltas) {
    int nx = x + dx, ny = y + dy;
    if (nx >= 0 && ny >= 0 && nx < costmap_->getSizeInCellsX() && ny < costmap_->getSizeInCellsY()) {
      if (costmap_->getCost(nx, ny) < nav2_costmap_2d::LETHAL_OBSTACLE) {  // Skip obstacle cells
        neighbors.emplace_back(nx, ny);  //append to the end of the vector
      }
    }
  }
  return neighbors;
}

// Heuristic function
double AStarPlanner::heuristic(int x1, int y1, int x2, int y2)
{
  return abs(x1 - x2) + abs(y1 - y2);  //Manhattan distance
}

// A* algorithm implementation - Create Plan virtual method
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
  
  //-- Initialization phase
  std::priority_queue<Node*, std::vector<Node*>, CompareNodes> open_set;
  // We also create a map to associate an ID to each node
  std::unordered_map<int, Node*> all_nodes;
  // Create the start node
  Node* start_node = new Node(start_x, start_y, 0, heuristic(start_x, start_y, goal_x, goal_y));
  // Put start node into the frontier
  open_set.push(start_node);
  // Here we use position of the node to compute a unique indentifier.
  //  This is a trick to rapidly retrieve the Node structure from its pose
  all_nodes[start_x * costmap_->getSizeInCellsY() + start_y] = start_node;

  // While frontier (queue) is not empty
  while (!open_set.empty()) {
    // Get best node
    Node* current = open_set.top();
    // Remove it from the queue
    open_set.pop();
    
    // If the current node is the goal, reconstruct the path
    if (current->x == goal_x && current->y == goal_y) {
      while (current) {
        geometry_msgs::msg::PoseStamped pose;
        double wx, wy;
        costmap_->mapToWorld(current->x, current->y, wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.orientation.z = goal.pose.orientation.z;
        pose.pose.orientation.w = goal.pose.orientation.w;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        global_path.poses.push_back(pose);
        current = current->parent;
      }
      std::reverse(global_path.poses.begin(), global_path.poses.end());
      return global_path;
    }
    
    // Expand the neighbors of the current node
    for (auto [nx, ny] : get_neighbors(current->x, current->y)) {
      double new_g = current->g + 1;
      //compute index of the expanded node
      int index = nx * costmap_->getSizeInCellsY() + ny;
      // If the neighbor has not been visited yet or has a lower cost
      if (!all_nodes.count(index) || new_g < all_nodes[index]->g) {
        // Add the node to the frontier (and to the map)
        Node* new_node = new Node(nx, ny, new_g, heuristic(nx, ny, goal_x, goal_y), current);
        open_set.push(new_node);
        all_nodes[index] = new_node;
      }
    }
  }
  return global_path;   // No path found
}

}  // namespace nav2_astar_planner

PLUGINLIB_EXPORT_CLASS(nav2_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)
