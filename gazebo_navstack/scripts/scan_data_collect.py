#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fyp_interfaces.msg import Mspeeds
import csv
import time

class ScanAnalysis(Node):

    def __init__(self):
        super().__init__('speed_subscriber')
        self.subscription = self.create_subscription(
            Mspeeds,
            '/Mspeeds',
            self.speed_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.csv_file = open('pid_data2.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'speeds'])  # Write header row
        self.count = 0

    def speed_callback(self, msg):
        # Save scan data with timestamp to CSV file
        timestamp = time.time()
        if msg.m1 != 0:
        	self.csv_writer.writerow([timestamp, msg.m1])
        
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
