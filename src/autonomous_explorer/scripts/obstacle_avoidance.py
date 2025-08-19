#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd = Twist()
        self.rate = rospy.Rate(10)
        self.turning = False

    def scan_callback(self, scan):
        # ✅ Corrected sector indices
        front = min(min(scan.ranges[0:15] + scan.ranges[-15:]), 10)
        left  = min(scan.ranges[75:105], 10)
        right = min(scan.ranges[255:285], 10)

        rospy.loginfo("Front: %.2f | Left: %.2f | Right: %.2f", front, left, right)

        safe_distance = 0.5  

        if front < safe_distance and not self.turning:
            # 🚨 Obstacle ahead → back up a little + rotate more
            self.turning = True
            self.reverse_and_turn(left, right)
        elif not self.turning:
            # ✅ Path clear → move forward
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd)

    def reverse_and_turn(self, left, right):
        # Step 1: small reverse
        self.cmd.linear.x = -0.2
        self.cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd)
        rospy.sleep(0.5)   # back for 0.5s

        # Step 2: stronger rotation
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.8 if right > left else -0.8
        self.cmd_vel_pub.publish(self.cmd)
        rospy.sleep(1.0)   # rotate for 1s

        self.turning = False  # allow normal driving again

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    robot = ObstacleAvoidance()
    robot.run()
