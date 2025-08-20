#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd = Twist()
        self.rate = rospy.Rate(10)

        # Parameters
        self.safe_distance = 0.6
        self.emergency_stop = 0.25
        self.forward_speed = 0.25
        self.turn_speed = 0.5

    def scan_callback(self, scan):
        ranges = np.array(scan.ranges)
        ranges[np.isnan(ranges)] = 10.0
        ranges[np.isinf(ranges)] = 10.0

        total = len(ranges)

        # Helper: convert angle to index
        def angle_to_index(angle_deg):
            angle_rad = math.radians(angle_deg)
            index = int((angle_rad - scan.angle_min) / scan.angle_increment)
            return max(0, min(total - 1, index))

        # Divide into regions
        front_angles = list(range(-15, 16))   # -15° to +15°
        left_angles  = list(range(45, 75))    # 45° to 75°
        right_angles = list(range(-75, -45))  # -75° to -45°

        def min_distance(angles):
            idxs = [angle_to_index(a) for a in angles]
            return min([ranges[i] for i in idxs])

        front = min_distance(front_angles)
        left  = min_distance(left_angles)
        right = min_distance(right_angles)

        rospy.loginfo("Front: %.2f | Left: %.2f | Right: %.2f", front, left, right)

        # --- Decision logic ---
        if front < self.emergency_stop:
            # Emergency halt
            self.stop()
            rospy.logwarn("!!! EMERGENCY STOP !!!")

        elif front < self.safe_distance:
            rospy.logwarn("Front blocked → turning")
            if right > left:
                self.turn(clockwise=True)
            else:
                self.turn(clockwise=False)

        elif left < self.safe_distance:
            rospy.logwarn("Too close to left wall → turning right")
            self.turn(clockwise=True)

        elif right < self.safe_distance:
            rospy.logwarn("Too close to right wall → turning left")
            self.turn(clockwise=False)

        else:
            self.move_forward()

    def move_forward(self):
        self.cmd.linear.x = self.forward_speed
        self.cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd)

    def stop(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd)

    def turn(self, clockwise=True):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = -self.turn_speed if clockwise else self.turn_speed
        self.cmd_vel_pub.publish(self.cmd)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    robot = ObstacleAvoidance()
    robot.run()
