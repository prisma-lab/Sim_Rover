#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np

class ReliableLaserMerger(Node):
    def __init__(self):
        super().__init__('scan_merger')
        
        # QoS Best Effort per input
        qos_in = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        
        # QoS Reliable per output
        qos_out = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        
        # Sottoscrittori
        self.sub_scan = Subscriber(self, LaserScan, '/scan', qos_profile=qos_in)
        self.sub_converted = Subscriber(self, LaserScan, '/converted_scan', qos_profile=qos_in)
        
        # Sincronizzatore
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_scan, self.sub_converted],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.merge_callback)
        
        # Publisher
        self.pub = self.create_publisher(LaserScan, '/merged_scan', qos_out)
        
        self.get_logger().info("✅ Reliable Laser Merger Started")
    
    def merge_callback(self, scan, converted):
        try:
            merged = LaserScan()
            merged.header = scan.header
            merged.header.frame_id = 'rover/lidar_link'
            
            # Copia parametri dallo scan principale
            merged.angle_min = scan.angle_min
            merged.angle_max = scan.angle_max
            merged.angle_increment = scan.angle_increment
            merged.time_increment = scan.time_increment
            merged.scan_time = scan.scan_time
            merged.range_min = min(scan.range_min, converted.range_min)
            merged.range_max = max(scan.range_max, converted.range_max)
            
            # Merge: prendi il minimo tra i due scan
            min_len = min(len(scan.ranges), len(converted.ranges))
            merged_ranges = []
            
            for i in range(min_len):
                scan_val = scan.ranges[i] if not np.isnan(scan.ranges[i]) else np.inf
                converted_val = converted.ranges[i] if not np.isnan(converted.ranges[i]) else np.inf
                merged_ranges.append(min(scan_val, converted_val))
            
            merged.ranges = merged_ranges
            self.pub.publish(merged)
            
            self.get_logger().info(f"📊 Published merged scan: {len(merged_ranges)} points")
            
        except Exception as e:
            self.get_logger().error(f"❌ Merge error: {e}")

def main():
    rclpy.init()
    node = ReliableLaserMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()