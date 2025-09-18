import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Polygon, Point32
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from collections import deque
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class CoverageNode(Node):
    def __init__(self):
        super().__init__('coverage_node')
        self.get_logger().info("Coverage Node Started")

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, 10)

        # Subscriber to robot position
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)

        # Subscriber to coverage area polygon
        self.polygon_sub = self.create_subscription(
            Polygon, '/coverage_area', self.polygon_callback, 10)

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Coverage parameters
        self.map_data = None
        self.robot_position = (0, 0)
        self.robot_orientation = 0.0
        self.coverage_area = None
        self.coverage_path = []
        self.current_waypoint_index = 0
        self.swath_width = 1.0  # Larghezza del passaggio (meters)
        self.navigation_in_progress = False
        self.coverage_active = False

        # Timer for coverage execution
        self.timer = self.create_timer(2.0, self.execute_coverage)

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received", throttle_duration_sec=5.0)

    def pose_callback(self, msg):
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)

    def polygon_callback(self, msg):
        """Riceve i 4 vertici del poligono di coverage"""
        if len(msg.points) != 4:
            self.get_logger().error("Il poligono deve avere esattamente 4 vertici")
            return

        self.coverage_area = []
        for point in msg.points:
            self.coverage_area.append((point.x, point.y))

        self.get_logger().info(f"Area di coverage ricevuta: {self.coverage_area}")
        self.generate_boustrophedon_path()
        self.coverage_active = True

    def generate_boustrophedon_path(self):
        """Genera un percorso bustrofedon all'interno del poligono"""
        if not self.coverage_area or len(self.coverage_area) != 4:
            self.get_logger().error("Area di coverage non valida")
            return

        # Converti i punti in numpy array per facilitare i calcoli
        points = np.array(self.coverage_area)
        
        # Trova l'angolo ottimale per il coverage
        optimal_angle = self.find_optimal_coverage_angle(points)
        
        # Ruota i punti per allineare il coverage lungo l'asse X
        rotated_points = self.rotate_points(points, -optimal_angle)
        
        # Trova i bounding box del poligono ruotato
        min_x, max_x = np.min(rotated_points[:, 0]), np.max(rotated_points[:, 0])
        min_y, max_y = np.min(rotated_points[:, 1]), np.max(rotated_points[:, 1])
        
        # Genera le linee di coverage
        y_positions = np.arange(min_y + self.swath_width/2, max_y, self.swath_width)
        waypoints = []
        
        for i, y in enumerate(y_positions):
            # Trova le intersezioni con il poligono
            intersections = self.find_polygon_intersections(rotated_points, y)
            
            if len(intersections) >= 2:
                # Ordina le intersezioni lungo l'asse X
                intersections.sort()
                x_start, x_end = intersections[0], intersections[-1]
                
                # Crea waypoints lungo questa linea
                if i % 2 == 0:  # Andata
                    waypoints.extend([
                        (x_start, y),
                        (x_end, y)
                    ])
                else:  # Ritorno
                    waypoints.extend([
                        (x_end, y),
                        (x_start, y)
                    ])
        
        # Ruota i waypoints di nuovo all'orientamento originale
        self.coverage_path = [self.rotate_point((x, y), optimal_angle) for x, y in waypoints]
        
        self.get_logger().info(f"Generato percorso con {len(self.coverage_path)} waypoints")
        self.current_waypoint_index = 0

    def find_optimal_coverage_angle(self, points):
        """Trova l'angolo ottimale per minimizzare i turni"""
        # Per semplicità, usa l'angolo del lato più lungo
        sides = [
            math.atan2(points[1][1]-points[0][1], points[1][0]-points[0][0]),
            math.atan2(points[2][1]-points[1][1], points[2][0]-points[1][0])
        ]
        return sides[0]  # Usa l'angolo del primo lato

    def rotate_points(self, points, angle):
        """Ruota i punti di un angolo specificato"""
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        rotated = []
        for x, y in points:
            rx = x * cos_angle - y * sin_angle
            ry = x * sin_angle + y * cos_angle
            rotated.append((rx, ry))
        return np.array(rotated)

    def rotate_point(self, point, angle):
        """Ruota un singolo punto"""
        x, y = point
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        return (
            x * cos_angle - y * sin_angle,
            x * sin_angle + y * cos_angle
        )

    def find_polygon_intersections(self, points, y):
        """Trova le intersezioni di una linea orizzontale con il poligono"""
        intersections = []
        n = len(points)
        
        for i in range(n):
            p1 = points[i]
            p2 = points[(i + 1) % n]
            
            # Controlla se la linea interseca questo segmento
            if (p1[1] <= y <= p2[1]) or (p2[1] <= y <= p1[1]):
                if p1[1] != p2[1]:  # Evita divisione per zero
                    t = (y - p1[1]) / (p2[1] - p1[1])
                    x = p1[0] + t * (p2[0] - p1[0])
                    intersections.append(x)
        
        return intersections

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Converte angoli di Eulero in quaternione"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def navigate_to(self, x, y):
        """Invia goal di navigazione"""
        if self.navigation_in_progress:
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        
        # Orientamento verso il prossimo waypoint
        if self.current_waypoint_index < len(self.coverage_path) - 1:
            next_x, next_y = self.coverage_path[self.current_waypoint_index + 1]
            yaw = math.atan2(next_y - y, next_x - x)
            qx, qy, qz, qw = self.euler_to_quaternion(0, 0, yaw)
            goal_msg.pose.orientation.x = qx
            goal_msg.pose.orientation.y = qy
            goal_msg.pose.orientation.z = qz
            goal_msg.pose.orientation.w = qw
        else:
            goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index}: ({x:.2f}, {y:.2f})")

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warning("Navigation server not available")
            return

        self.navigation_in_progress = True
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            self.navigation_in_progress = False
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed")
            
            # Passa al prossimo waypoint
            self.current_waypoint_index += 1
            self.navigation_in_progress = False

        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
            self.navigation_in_progress = False

    def execute_coverage(self):
        """Esegue il coverage waypoint per waypoint"""
        if not self.coverage_active or self.navigation_in_progress:
            return

        if self.current_waypoint_index >= len(self.coverage_path):
            self.get_logger().info("Coverage completato!")
            self.coverage_active = False
            return

        # Naviga verso il prossimo waypoint
        waypoint = self.coverage_path[self.current_waypoint_index]
        self.navigate_to(waypoint[0], waypoint[1])

    def is_point_in_polygon(self, point, polygon):
        """Controlla se un punto è dentro il poligono"""
        x, y = point
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

def main(args=None):
    rclpy.init(args=args)
    coverage_node = CoverageNode()

    try:
        coverage_node.get_logger().info("Coverage Node Ready")
        coverage_node.get_logger().info("Inviare un messaggio Polygon su /coverage_area per iniziare")
        rclpy.spin(coverage_node)
    except KeyboardInterrupt:
        coverage_node.get_logger().info("Coverage stopped by user")
    finally:
        coverage_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()