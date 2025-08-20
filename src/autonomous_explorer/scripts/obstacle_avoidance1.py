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
        
        # Add state tracking for smarter behavior
        self.last_turn_direction = 1  # 1 for left, -1 for right
        self.turn_counter = 0
        self.log_counter = 0

    def scan_callback(self, scan):
        # --- Your original indexes (working fine) ---
        # Front at 180°, Left at 90°, Right at 270°
        front = scan.ranges[180]
        left  = scan.ranges[90]
        right = scan.ranges[270]

        # Enhanced safety clamping with NaN check
        front = min(front, 10.0) if front != float('inf') and front == front else 10.0
        left  = min(left, 10.0) if left != float('inf') and left == left else 10.0
        right = min(right, 10.0) if right != float('inf') and right == right else 10.0

        # Reduce logging spam - log every 5th reading
        self.log_counter += 1
        if self.log_counter % 5 == 0:
            rospy.loginfo("Front: %.2f | Left: %.2f | Right: %.2f", front, left, right)

        # --- Enhanced Parameters ---
        safe_distance = 0.8     # Increased for more safety margin
        emergency_stop = 0.3    # Slightly increased
        max_forward_speed = 0.25  # Reasonable max speed
        turn_speed = 0.6        # Effective turn speed

        # --- Improved obstacle avoidance logic ---
        if front < emergency_stop:
            # Emergency stop
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            rospy.logwarn("!!! EMERGENCY STOP !!!")
            
            # Set turn direction for when we start turning
            self.last_turn_direction = 1 if right > left else -1
            self.turn_counter = 0
            
        elif front < safe_distance:
            # Obstacle ahead - enhanced turning logic
            self.cmd.linear.x = 0.0
            
            # For first few turns, stick with chosen direction for consistency
            if self.turn_counter < 8:
                # Use the direction that has more space, but be consistent
                if self.turn_counter == 0:  # First turn - choose direction
                    self.last_turn_direction = 1 if right > left else -1
                self.cmd.angular.z = turn_speed * self.last_turn_direction
            else:
                # After several turns, reassess if we're stuck
                self.last_turn_direction = 1 if right > left else -1
                self.cmd.angular.z = turn_speed * self.last_turn_direction
                self.turn_counter = 0  # Reset counter
            
            self.turn_counter += 1
            
        else:
            # Path is clear - adaptive forward movement
            self.turn_counter = 0
            
            # Adaptive speed based on front distance
            if front > safe_distance * 2:
                # Lots of space - go faster
                self.cmd.linear.x = max_forward_speed
            else:
                # Some space - go slower for safety
                speed_factor = (front - safe_distance) / safe_distance
                self.cmd.linear.x = max_forward_speed * max(0.4, min(1.0, speed_factor))
            
            # Gentle course correction to avoid getting too close to side walls
            if left < safe_distance * 1.2 and right > safe_distance * 1.2:
                # Left wall close - turn slightly right
                self.cmd.angular.z = -0.15
            elif right < safe_distance * 1.2 and left > safe_distance * 1.2:
                # Right wall close - turn slightly left  
                self.cmd.angular.z = 0.15
            else:
                # Go straight
                self.cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(self.cmd)

    def run(self):
        rospy.loginfo("Obstacle avoidance started - ready to navigate!")
        while not rospy.is_shutdown():
            self.rate.sleep()
        
        # Stop robot on shutdown
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.loginfo("Obstacle avoidance stopped")

if __name__ == '__main__':
    try:
        robot = ObstacleAvoidance()
        robot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted")
    except Exception as e:
        rospy.logerr("Error: %s", str(e))