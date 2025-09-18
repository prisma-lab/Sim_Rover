#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        
        # QoS profile BEST EFFORT
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.sub_lidar = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile
        )
        
        self.sub_camera = self.create_subscription(
            LaserScan, '/converted_scan', self.camera_callback, qos_profile
        )
        
        # Publisher
        self.pub = self.create_publisher(LaserScan, '/merged_scan', qos_profile)
        
        # Data storage
        self.lidar_scan = None
        self.camera_scan = None
        
        # Parametri configurabili
        self.lidar_max_range = 40.0  # Range massimo realistico del lidar
        self.camera_max_range = 20.  # Range massimo realistico della camera
        
        self.get_logger().info("Scan Merger Node Started con correzioni")
    
    def lidar_callback(self, msg):
        self.lidar_scan = self.clean_lidar_scan(msg)
        self.merge_and_publish()
    
    def camera_callback(self, msg):
        self.camera_scan = self.fix_camera_scan_orientation(msg)
        self.merge_and_publish()
    
    def clean_lidar_scan(self, scan):
        """Pulisce lo scan del lidar dai valori 'inf' ai confini"""
        cleaned_scan = LaserScan()
        cleaned_scan.header = scan.header
        cleaned_scan.angle_min = scan.angle_min
        cleaned_scan.angle_max = scan.angle_max
        cleaned_scan.angle_increment = scan.angle_increment
        cleaned_scan.time_increment = scan.time_increment
        cleaned_scan.scan_time = scan.scan_time
        cleaned_scan.range_min = scan.range_min
        cleaned_scan.range_max = scan.range_max
        
        # Sostituisce gli 'inf' con il range_max realistico
        cleaned_ranges = []
        for r in scan.ranges:
            if r == float('inf') or r > self.lidar_max_range:
                cleaned_ranges.append(self.lidar_max_range)
            else:
                cleaned_ranges.append(r)
        
        cleaned_scan.ranges = cleaned_ranges
        return cleaned_scan
    
    def fix_camera_scan_orientation(self, scan):
        """Corregge l'orientamento invertito dello scan della camera"""
        fixed_scan = LaserScan()
        fixed_scan.header = scan.header
        fixed_scan.header.frame_id = 'rover/lidar_link'  # Usa stesso frame del lidar
        
        # Inverte l'orientamento angolare
        fixed_scan.angle_min = -scan.angle_max  # Inverti min/max
        fixed_scan.angle_max = -scan.angle_min
        fixed_scan.angle_increment = scan.angle_increment
        fixed_scan.time_increment = scan.time_increment
        fixed_scan.scan_time = scan.scan_time
        fixed_scan.range_min = scan.range_min
        fixed_scan.range_max = scan.range_max
        
        # Inverte l'ordine dei ranges
        fixed_scan.ranges = list(reversed(scan.ranges))
        
        return fixed_scan
    
    def merge_and_publish(self):
        if self.lidar_scan is None or self.camera_scan is None:
            return
        
        try:
            merged_scan = LaserScan()
            merged_scan.header = self.lidar_scan.header
            merged_scan.header.frame_id = 'rover/lidar_link'
            merged_scan.angle_min = self.lidar_scan.angle_min
            merged_scan.angle_max = self.lidar_scan.angle_max
            merged_scan.angle_increment = self.lidar_scan.angle_increment
            merged_scan.time_increment = self.lidar_scan.time_increment
            merged_scan.scan_time = self.lidar_scan.scan_time
            merged_scan.range_min = min(self.lidar_scan.range_min, self.camera_scan.range_min)
            merged_scan.range_max = max(self.lidar_scan.range_max, self.camera_scan.range_max)
            
            # Usa i valori del lidar come base
            merged_scan.ranges = list(self.lidar_scan.ranges)
            
            # Merge intelligente con la camera
            camera_ranges = self.interpolate_camera_scan()
            
            for i in range(len(merged_scan.ranges)):
                if i < len(camera_ranges):
                    camera_range = camera_ranges[i]
                    lidar_range = merged_scan.ranges[i]
                    
                    # Sostituisci solo se:
                    # 1. La camera vede un ostacolo più vicino
                    # 2. Il valore della camera è realistico (non inf o troppo alto)
                    # 3. Il lidar non sta vedendo nulla di utile (range troppo alto)
                    if (camera_range < lidar_range and 
                        camera_range >= merged_scan.range_min and 
                        camera_range <= self.camera_max_range and
                        (lidar_range > self.lidar_max_range * 0.8 or lidar_range == float('inf'))):
                        merged_scan.ranges[i] = camera_range
            
            self.pub.publish(merged_scan)
            
        except Exception as e:
            self.get_logger().error(f"Error in merge: {str(e)}")
    
    def interpolate_camera_scan(self):
        """Interpola lo scan della camera per matchare la risoluzione del lidar"""
        if not self.camera_scan.ranges:
            return []
        
        lidar_resolution = len(self.lidar_scan.ranges)
        camera_resolution = len(self.camera_scan.ranges)
        
        if camera_resolution == 0:
            return []
        
        # Interpolazione più accurata
        interpolated = []
        lidar_angles = np.linspace(self.lidar_scan.angle_min, self.lidar_scan.angle_max, lidar_resolution)
        camera_angles = np.linspace(self.camera_scan.angle_min, self.camera_scan.angle_max, camera_resolution)
        
        for lidar_angle in lidar_angles:
            # Trova l'indice più vicino nella camera
            angle_diff = np.abs(camera_angles - lidar_angle)
            closest_idx = np.argmin(angle_diff)
            
            if closest_idx < len(self.camera_scan.ranges):
                interpolated.append(self.camera_scan.ranges[closest_idx])
            else:
                interpolated.append(float('inf'))
        
        return interpolated

def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()