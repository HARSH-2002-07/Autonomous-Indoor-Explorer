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

    def scan_callback(self, scan):
        # --- Corrected indexes based on your scan orientation ---
        # Front at 180°, Left at 90°, Right at 270°
        front = scan.ranges[180]
        left  = scan.ranges[90]
        right = scan.ranges[270]

        # Safety clamp for inf/nan values
        front = min(front, 10.0) if front != float('inf') else 10.0
        left  = min(left, 10.0) if left != float('inf') else 10.0
        right = min(right, 10.0) if right != float('inf') else 10.0

        rospy.loginfo("Front: %.2f | Left: %.2f | Right: %.2f", front, left, right)

        # --- Parameters ---
        safe_distance = 0.6    # Minimum distance before stopping
        emergency_stop = 0.25  # Hard stop if too close

        # --- Obstacle avoidance logic ---
        if front < emergency_stop:
            # Immediate emergency stop
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            rospy.logwarn("!!! EMERGENCY STOP !!!")
        elif front < safe_distance:
            # Obstacle detected ahead → stop & turn
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5 if right > left else -0.5
        else:
            # Path is clear → move forward
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(self.cmd)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    robot = ObstacleAvoidance()
    robot.run()
