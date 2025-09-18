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
        self.latest_lidar_scan = None
        self.latest_camera_scan = None
        
        # Parametri configurabili
        self.lidar_max_range = 40.0
        self.camera_max_range = 20.0
        self.publish_frequency = 10.0  # 10 Hz
        
        # Timer per pubblicazione a frequenza fissa
        self.timer = self.create_timer(
            1.0 / self.publish_frequency,
            self.timer_callback
        )
        
        self.get_logger().info(f"Scan Merger Node Started at {self.publish_frequency} Hz")
    
    def lidar_callback(self, msg):
        """Salva l'ultimo scan del lidar"""
        self.latest_lidar_scan = msg
    
    def camera_callback(self, msg):
        """Salva l'ultimo scan della camera (già nel frame corretto)"""
        self.latest_camera_scan = msg
    
    def timer_callback(self):
        """Callback del timer a 10 Hz - elabora e pubblica"""
        if self.latest_lidar_scan is None or self.latest_camera_scan is None:
            self.get_logger().debug("Waiting for both scans...")
            return
        
        try:
            # Pulisci il lidar scan
            lidar_scan = self.clean_lidar_scan(self.latest_lidar_scan)
            
            # La camera scan è già corretta, usa direttamente
            camera_scan = self.prepare_camera_scan(self.latest_camera_scan)
            
            # Merge e pubblica
            merged_scan = self.merge_scans(lidar_scan, camera_scan)
            self.pub.publish(merged_scan)
            
        except Exception as e:
            self.get_logger().error(f"Error in processing: {str(e)}")
    
    def clean_lidar_scan(self, scan):
        """Pulisce lo scan del lidar"""
        cleaned_scan = LaserScan()
        cleaned_scan.header = scan.header
        cleaned_scan.header.stamp = self.get_clock().now().to_msg()
        cleaned_scan.angle_min = scan.angle_min
        cleaned_scan.angle_max = scan.angle_max
        cleaned_scan.angle_increment = scan.angle_increment
        cleaned_scan.time_increment = scan.time_increment
        cleaned_scan.scan_time = 1.0 / self.publish_frequency
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
    
    def prepare_camera_scan(self, scan):
        """Prepara lo scan della camera (già nel frame corretto)"""
        prepared_scan = LaserScan()
        prepared_scan.header = scan.header
        prepared_scan.header.stamp = self.get_clock().now().to_msg()
        prepared_scan.header.frame_id = 'rover/lidar_link'  # Conferma frame
        
        # Mantieni tutti i parametri originali (NESSuna rotazione)
        prepared_scan.angle_min = scan.angle_min
        prepared_scan.angle_max = scan.angle_max
        prepared_scan.angle_increment = scan.angle_increment
        prepared_scan.time_increment = scan.time_increment
        prepared_scan.scan_time = 1.0 / self.publish_frequency
        prepared_scan.range_min = scan.range_min
        prepared_scan.range_max = scan.range_max
        
        # Usa i ranges originali (NESSuna inversione)
        prepared_scan.ranges = list(scan.ranges)
        
        return prepared_scan
    
    def merge_scans(self, lidar_scan, camera_scan):
        """Fonde i due scans"""
        merged_scan = LaserScan()
        merged_scan.header = lidar_scan.header
        merged_scan.header.frame_id = 'rover/lidar_link'
        merged_scan.angle_min = lidar_scan.angle_min
        merged_scan.angle_max = lidar_scan.angle_max
        merged_scan.angle_increment = lidar_scan.angle_increment
        merged_scan.time_increment = lidar_scan.time_increment
        merged_scan.scan_time = lidar_scan.scan_time
        merged_scan.range_min = min(lidar_scan.range_min, camera_scan.range_min)
        merged_scan.range_max = max(lidar_scan.range_max, camera_scan.range_max)
        
        # Usa i valori del lidar come base
        merged_scan.ranges = list(lidar_scan.ranges)
        
        # Merge intelligente con la camera
        camera_ranges = self.interpolate_camera_scan(lidar_scan, camera_scan)
        
        for i in range(len(merged_scan.ranges)):
            if i < len(camera_ranges):
                camera_range = camera_ranges[i]
                lidar_range = merged_scan.ranges[i]
                
                # Sostituisci solo se la camera vede un ostacolo più vicino e realistico
                if (camera_range < lidar_range and 
                    camera_range >= merged_scan.range_min and 
                    camera_range <= self.camera_max_range):
                    merged_scan.ranges[i] = camera_range
        
        return merged_scan
    
    def interpolate_camera_scan(self, lidar_scan, camera_scan):
        """Interpola lo scan della camera per matchare la risoluzione del lidar"""
        if not camera_scan.ranges:
            return []
        
        lidar_resolution = len(lidar_scan.ranges)
        camera_resolution = len(camera_scan.ranges)
        
        if camera_resolution == 0:
            return []
        
        # Interpolazione angolare semplice
        interpolated = []
        lidar_angles = np.linspace(lidar_scan.angle_min, lidar_scan.angle_max, lidar_resolution)
        
        for lidar_angle in lidar_angles:
            # Trova l'indice corrispondente nella camera
            camera_angle = lidar_angle
            
            # Normalizza l'angolo tra -π e π
            while camera_angle > np.pi:
                camera_angle -= 2 * np.pi
            while camera_angle < -np.pi:
                camera_angle += 2 * np.pi
            
            # Calcola l'indice
            if camera_angle < camera_scan.angle_min or camera_angle > camera_scan.angle_max:
                interpolated.append(float('inf'))
                continue
                
            camera_idx = int((camera_angle - camera_scan.angle_min) / camera_scan.angle_increment)
            
            if 0 <= camera_idx < len(camera_scan.ranges):
                interpolated.append(camera_scan.ranges[camera_idx])
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