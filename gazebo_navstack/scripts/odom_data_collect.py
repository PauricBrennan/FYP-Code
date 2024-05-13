#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import time

class ScanAnalysis(Node):

    def __init__(self):
        super().__init__('speed_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.csv_file = open('odom_data2.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'X', 'Y', 'VX', 'VY', 'VYAW'])  # Write header row
        self.count = 0

    def odom_callback(self, msg):
        # Save scan data with timestamp to CSV file
        timestamp = time.time()
        X = msg.pose.pose.position.x
        Y = msg.pose.pose.position.y
        VX = msg.twist.twist.linear.x
        VY = msg.twist.twist.linear.y
        VYAW = msg.twist.twist.angular.z
        self.csv_writer.writerow([timestamp, X, Y, VX, VY, VYAW])
        
    def close(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    scan = ScanAnalysis()
    rclpy.spin(scan)
    scan.close()
    scan.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
