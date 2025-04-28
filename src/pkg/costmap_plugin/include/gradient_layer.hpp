#ifndef NAV2_COSTMAP_2D__gradient_LAYER_HPP_
#define NAV2_COSTMAP_2D__gradient_LAYER_HPP_

#include <map>
#include <vector>
#include <mutex>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace costmap_plugin
{
/**
 * @class CellData
 * @brief Storage for cell information used during obstacle gradient
 */
class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(unsigned int x, unsigned int y, unsigned int sx, unsigned int sy)
  : x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

/**
 * @class GradientLayer
 * @brief Layer to convolve costmap by robot's radius or footprint to prevent
 * collisions and largely simply collision checking
 */
class GradientLayer : public nav2_costmap_2d::Layer
{
public:
  /**
    * @brief A constructor
    */
  GradientLayer();

  /**
    * @brief A destructor
    */
  ~GradientLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  void onInitialize() override;

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;
  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief Match the size of the master costmap
   */
  void matchSize() override;

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  bool isClearable() override {return false;}

  /**
   * @brief Reset this costmap
   */
  void reset() override
  {
    matchSize();
    current_ = false;
  }

  /** @brief  Given a distance, compute a cost.
   * @param  distance The distance from the nearest obstacle in cells
   * @return A cost value for the distance */
  inline unsigned char computeCost(double distance) const
  {
    if (distance == 0.0) {
      return nav2_costmap_2d::LETHAL_OBSTACLE;
    }
  
    double max_distance = gradient_radius_ / resolution_;  // maximum inflation distance in cells
  
    if (distance >= max_distance) {
      return nav2_costmap_2d::FREE_SPACE;
    }
  
  // Linearly decay the cost between LETHAL_OBSTACLE and FREE_SPACE
    double ratio = (max_distance - distance) / max_distance;
    unsigned char cost = static_cast<unsigned char>(
      nav2_costmap_2d::FREE_SPACE + ratio * (nav2_costmap_2d::LETHAL_OBSTACLE - nav2_costmap_2d::FREE_SPACE));
  
    return cost;
  }
  
  /**
   * @brief Retrieve a pointer to the GradientLayer from the layered costmap.
   * @param costmap_ros Shared pointer to the Costmap2DROS managing the layered costmap.
   * @return Shared pointer to the GradientLayer if found, nullptr otherwise.
   */
  static std::shared_ptr<costmap_plugin::GradientLayer> getGradientLayer(
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros,
    const std::string layer_name = "")
  {
    const auto layered_costmap = costmap_ros->getLayeredCostmap();
    for (auto layer = layered_costmap->getPlugins()->begin();
      layer != layered_costmap->getPlugins()->end();
      ++layer)
    {
      auto gradient_layer = std::dynamic_pointer_cast<costmap_plugin::GradientLayer>(*layer);
      if (gradient_layer) {
        if (layer_name.empty() || gradient_layer->getName() == layer_name) {
          return gradient_layer;
        }
      }
    }
    return nullptr;
  }

  // Provide a typedef to ease future code maintenance
  typedef std::recursive_mutex mutex_t;

  /**
   * @brief Get the mutex of the gradient information
   */
  mutex_t * getMutex()
  {
    return access_;
  }
  /**
   * @brief Get the cost scaling factor
   */
  double getCostScalingFactor()
  {
    return cost_scaling_factor_;
  }

  double getgradientRadius()
  {
    return gradient_radius_;
  }

protected:
  /**
   * @brief Process updates on footprint changes to the gradient layer
   */
  void onFootprintChanged() override;

  /**
   * @brief  Lookup pre-computed distances between current and source cells with cache
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline double distanceLookup(
    unsigned int mx, unsigned int my, unsigned int src_x,
    unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_distances_[dx * cache_length_ + dy];
  }

  /**
   * @brief  Lookup pre-computed costs
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline unsigned char costLookup(
    unsigned int mx, unsigned int my, unsigned int src_x,
    unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_costs_[dx * cache_length_ + dy];
  }

  /**
   * @brief Compute cached dsitances
   */
  void computeCaches();

  /**
   * @brief Compute cached dsitances
   */
  int generateIntegerDistances();

  /**
   * @brief Compute cached dsitances
   */
  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  /**
   * @brief Enqueue new cells in cache distance update search
   */
  inline void enqueue(
    unsigned int index, unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  double gradient_radius_, inscribed_radius_, cost_scaling_factor_;
  bool inflate_unknown_, inflate_around_unknown_;
  unsigned int cell_gradient_radius_;
  unsigned int cached_cell_gradient_radius_;
  std::vector<std::vector<CellData>> gradient_cells_;

  double resolution_;

  std::vector<bool> seen_;

  std::vector<unsigned char> cached_costs_;
  std::vector<double> cached_distances_;
  std::vector<std::vector<int>> distance_matrix_;
  unsigned int cache_length_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire costmap should be reinflated next time around.
  bool need_regradient_;
  mutex_t * access_;
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__gradient_LAYER_HPP_