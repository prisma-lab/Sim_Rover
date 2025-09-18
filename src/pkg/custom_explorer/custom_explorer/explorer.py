import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from collections import deque
import math


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, 10)

        # Subscriber to robot position (amcl_pose or similar)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Map and position data
        self.map_data = None
        self.robot_position = (0, 0)  # (x, y) in world coordinates
        self.robot_orientation = 0.0

        # Sector exploration parameters
        self.sector_size = 2.0  # meters for each sector
        self.sectors = {}  # Dictionary to track sector exploration status
        self.sector_queue = deque()  # Queue of sectors to explore
        self.current_goal_sector = None
        self.current_goal_id = None
        self.sectors_initialized = False  # Flag to track if sectors are already initialized

        # Timer for periodic exploration
        self.timer = self.create_timer(5.0, self.explore_sectors)
        
        # Flag to check if navigation is in progress
        self.navigation_in_progress = False

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received", throttle_duration_sec=5.0)  # Throttle log messages
        
        # Initialize sectors only once when first map is received
        if not self.sectors_initialized:
            self.initialize_sectors()
            self.sectors_initialized = True
        else:
            # If sectors are already initialized, just update the map data
            # without resetting the exploration progress
            self.get_logger().debug("Map updated, but sectors already initialized")

    def pose_callback(self, msg):
        # Update robot position from localization
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)

    def initialize_sectors(self):
        """Divide the map into sectors based on fixed distance"""
        if self.map_data is None:
            return

        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y
        width = self.map_data.info.width * self.map_data.info.resolution
        height = self.map_data.info.height * self.map_data.info.resolution

        # Calculate number of sectors in x and y directions
        sectors_x = int(math.ceil(width / self.sector_size))
        sectors_y = int(math.ceil(height / self.sector_size))

        self.sectors = {}
        self.sector_queue = deque()

        for i in range(sectors_x):
            for j in range(sectors_y):
                # Calculate sector center in world coordinates
                sector_center_x = origin_x + (i + 0.5) * self.sector_size
                sector_center_y = origin_y + (j + 0.5) * self.sector_size
                
                # Check if sector center is within map bounds and not occupied
                if self.is_valid_sector(sector_center_x, sector_center_y):
                    sector_id = f"{i}_{j}"
                    self.sectors[sector_id] = {
                        'center_x': sector_center_x,
                        'center_y': sector_center_y,
                        'explored': False,
                        'coordinates': (i, j)
                    }
                    self.sector_queue.append(sector_id)

        self.get_logger().info(f"Initialized {len(self.sectors)} sectors")

    def is_valid_sector(self, x, y):
        """Check if sector center is valid (not occupied and within map)"""
        if self.map_data is None:
            return False

        # Convert world coordinates to map coordinates
        map_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        map_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Check if within map bounds
        if (map_x < 0 or map_x >= self.map_data.info.width or 
            map_y < 0 or map_y >= self.map_data.info.height):
            return False

        # Check if not occupied (0 = free, -1 = unknown, >0 = occupied)
        map_index = map_y * self.map_data.info.width + map_x
        if (map_index < len(self.map_data.data) and 
            self.map_data.data[map_index] > 50):  # Threshold for occupied
            return False

        return True

    def get_closest_unexplored_sector(self):
        """Find the closest unexplored sector to the robot"""
        closest_sector = None
        min_distance = float('inf')

        for sector_id, sector_data in self.sectors.items():
            if not sector_data['explored']:
                distance = math.sqrt(
                    (self.robot_position[0] - sector_data['center_x'])**2 +
                    (self.robot_position[1] - sector_data['center_y'])**2
                )
                if distance < min_distance:
                    min_distance = distance
                    closest_sector = sector_id

        return closest_sector

    def navigate_to(self, x, y):
        """Send navigation goal to Nav2"""
        if self.navigation_in_progress:
            self.get_logger().info("Navigation already in progress, skipping new goal")
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0  # Facing forward

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to sector center: x={x:.2f}, y={y:.2f}")

        # Wait for the action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().warning("Navigation server not available")
            return

        # Send the goal and register a callback for the result
        self.navigation_in_progress = True
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the goal response"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            self.navigation_in_progress = False
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        """Callback to handle the result of the navigation action"""
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed with result: {result}")
            
            # Mark current sector as explored
            if self.current_goal_sector:
                self.sectors[self.current_goal_sector]['explored'] = True
                self.get_logger().info(f"Sector {self.current_goal_sector} marked as explored")
            
            self.navigation_in_progress = False
            self.current_goal_sector = None

        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
            self.navigation_in_progress = False
            self.current_goal_sector = None

    def explore_sectors(self):
        """Explore sectors periodically"""
        if self.map_data is None:
            self.get_logger().warning("No map data available")
            return

        if self.navigation_in_progress:
            self.get_logger().info("Navigation in progress, skipping exploration cycle")
            return

        # Find closest unexplored sector
        closest_sector = self.get_closest_unexplored_sector()

        if not closest_sector:
            self.get_logger().info("All sectors explored! Exploration complete!")
            return

        # Get sector data
        sector_data = self.sectors[closest_sector]
        
        # Navigate to sector center
        self.current_goal_sector = closest_sector
        self.navigate_to(sector_data['center_x'], sector_data['center_y'])

    def is_robot_in_sector(self, sector_id):
        """Check if robot is currently in the specified sector"""
        if sector_id not in self.sectors:
            return False
            
        sector_data = self.sectors[sector_id]
        sector_min_x = sector_data['center_x'] - self.sector_size / 2
        sector_max_x = sector_data['center_x'] + self.sector_size / 2
        sector_min_y = sector_data['center_y'] - self.sector_size / 2
        sector_max_y = sector_data['center_y'] + self.sector_size / 2
        
        return (sector_min_x <= self.robot_position[0] <= sector_max_x and
                sector_min_y <= self.robot_position[1] <= sector_max_y)


def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        explorer_node.get_logger().info("Starting sector-based exploration...")
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()