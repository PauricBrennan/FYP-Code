#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import csv
import time

class ScanFix(Node):

	def __init__(self):
		super().__init__('scan_subscriber')
		self.subscription = self.create_subscription(
		LaserScan,
		'/yd/scan',
		self.scan_callback,
		rclpy.qos.qos_profile_sensor_data
		)
        
		self.publisher = self.create_publisher(LaserScan, '/scan', 10)
		self.current_scan = LaserScan()
		self.prev1_scan = []
		self.prev2_scan = []
		self.count = 0

	def scan_callback(self, msg):
		#self.current_scan = msg
		
		
		if self.count != 10:
			self.count = self.count + 1
		else:
			for i, value in enumerate(msg.ranges):
				if value == 0.0 and i != 230:
					msg.ranges[i] = (self.prev1_scan[i] + self.prev2_scan[i]) / 2
					#print(self.prev2_scan[5])
			for i, value in enumerate(msg.ranges):
				if value > 4 and i != 230:
					msg.ranges[i] = 0.0
					#print(self.prev2_scan[5])
					
			self.publisher.publish(msg)
    	
		self.prev2_scan = self.prev1_scan
		self.prev1_scan = msg.ranges

def main(args=None):
	rclpy.init(args=args)
	scan = ScanFix()
	rclpy.spin(scan)
	scan.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
