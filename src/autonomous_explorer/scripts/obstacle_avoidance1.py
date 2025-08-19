#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from collections import deque
import math

class OptimizedObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        
        # Parameters (can be set via ROS parameter server)
        self.safe_distance = rospy.get_param('~safe_distance', 0.8)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.3)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 1.5)
        
        # State variables
        self.cmd = Twist()
        self.last_scan_time = rospy.Time.now()
        self.scan_timeout = rospy.Duration(1.0)  # 1 second timeout
        
        # Smoothing filter for commands
        self.cmd_history = deque(maxlen=3)
        
        # Pre-calculate angle indices for efficiency
        self.angle_indices = {}
        
        rospy.loginfo("Optimized Obstacle Avoidance Node Initialized")
        rospy.loginfo("Safe distance: %.2f m", self.safe_distance)
    
    def calculate_angle_indices(self, scan):
        """Pre-calculate angle indices based on scan parameters"""
        if self.angle_indices:
            return  # Already calculated
            
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        ranges_len = len(scan.ranges)
        
        def angle_to_index(angle):
            """Convert angle to array index"""
            idx = int((angle - angle_min) / angle_increment)
            return max(0, min(idx, ranges_len - 1))
        
        # Define regions with better coverage
        self.angle_indices = {
            'front': list(range(angle_to_index(-math.pi/6), angle_to_index(math.pi/6))),
            'front_left': list(range(angle_to_index(math.pi/6), angle_to_index(math.pi/3))),
            'front_right': list(range(angle_to_index(-math.pi/3), angle_to_index(-math.pi/6))),
            'left': list(range(angle_to_index(math.pi/3), angle_to_index(2*math.pi/3))),
            'right': list(range(angle_to_index(-2*math.pi/3), angle_to_index(-math.pi/3)))
        }
        
        # Handle wraparound for front region
        if angle_to_index(-math.pi/6) > angle_to_index(math.pi/6):
            front_indices = (list(range(0, angle_to_index(math.pi/6))) + 
                           list(range(angle_to_index(-math.pi/6), ranges_len)))
            self.angle_indices['front'] = front_indices
    
    def get_region_distance(self, ranges, region_key):
        """Get minimum valid distance in a region"""
        indices = self.angle_indices[region_key]
        valid_ranges = []
        
        for i in indices:
            if i < len(ranges) and not (math.isnan(ranges[i]) or math.isinf(ranges[i])):
                valid_ranges.append(ranges[i])
        
        return min(valid_ranges) if valid_ranges else float('inf')
    
    def calculate_repulsive_force(self, ranges):
        """Calculate repulsive force vector from nearby obstacles"""
        force_x, force_y = 0.0, 0.0
        angle_min = -math.pi  # Assuming full 360° scan
        angle_increment = 2 * math.pi / len(ranges)
        
        for i, distance in enumerate(ranges):
            if math.isnan(distance) or math.isinf(distance) or distance > self.lookahead_distance:
                continue
                
            if distance < self.safe_distance:
                angle = angle_min + i * angle_increment
                # Stronger repulsion for closer obstacles
                force_magnitude = (self.safe_distance - distance) / self.safe_distance
                force_x -= force_magnitude * math.cos(angle)
                force_y -= force_magnitude * math.sin(angle)
        
        return force_x, force_y
    
    def smooth_command(self, new_cmd):
        """Apply smoothing filter to commands"""
        self.cmd_history.append((new_cmd.linear.x, new_cmd.angular.z))
        
        if len(self.cmd_history) < 2:
            return new_cmd
        
        # Simple moving average
        avg_linear = sum(cmd[0] for cmd in self.cmd_history) / len(self.cmd_history)
        avg_angular = sum(cmd[1] for cmd in self.cmd_history) / len(self.cmd_history)
        
        smoothed_cmd = Twist()
        smoothed_cmd.linear.x = avg_linear
        smoothed_cmd.angular.z = avg_angular
        
        return smoothed_cmd
    
    def scan_callback(self, scan):
        """Process laser scan data and generate movement commands"""
        self.last_scan_time = rospy.Time.now()
        
        # Initialize angle indices on first scan
        self.calculate_angle_indices(scan)
        
        # Convert to numpy array for efficient processing
        ranges = np.array(scan.ranges)
        
        # Replace inf and nan with max range
        ranges = np.where(np.isfinite(ranges), ranges, scan.range_max)
        
        # Get distances for each region
        regions = {}
        for region in self.angle_indices.keys():
            regions[region] = self.get_region_distance(ranges, region)
        
        # Calculate repulsive forces
        force_x, force_y = self.calculate_repulsive_force(ranges)
        
        # Log region distances (less frequently for performance)
        if rospy.get_time() % 2 < 0.1:  # Log every 2 seconds
            rospy.loginfo("Regions - Front: %.2f | Left: %.2f | Right: %.2f", 
                         regions['front'], regions['left'], regions['right'])
        
        # Decision making with improved logic
        new_cmd = Twist()
        
        if regions['front'] < self.safe_distance:
            # Emergency stop and turn
            new_cmd.linear.x = 0.0
            
            # Choose turn direction based on available space and forces
            if abs(force_y) > 0.1:
                new_cmd.angular.z = self.max_angular_speed if force_y > 0 else -self.max_angular_speed
            else:
                # Fallback to comparing sides
                turn_right = regions['right'] > regions['left']
                new_cmd.angular.z = -self.max_angular_speed if turn_right else self.max_angular_speed
                
        elif regions['front_left'] < self.safe_distance or regions['front_right'] < self.safe_distance:
            # Partial obstacle - slow down and adjust
            new_cmd.linear.x = self.max_linear_speed * 0.5
            
            if regions['front_right'] < regions['front_left']:
                new_cmd.angular.z = self.max_angular_speed * 0.5  # Turn left
            else:
                new_cmd.angular.z = -self.max_angular_speed * 0.5  # Turn right
                
        else:
            # Clear path - move forward with potential minor corrections
            new_cmd.linear.x = self.max_linear_speed
            
            # Small corrections based on side clearances
            if regions['left'] < self.safe_distance * 1.5:
                new_cmd.angular.z = -0.2  # Slight right turn
            elif regions['right'] < self.safe_distance * 1.5:
                new_cmd.angular.z = 0.2   # Slight left turn
            else:
                new_cmd.angular.z = 0.0
        
        # Apply command smoothing
        smoothed_cmd = self.smooth_command(new_cmd)
        
        # Publish command
        self.cmd_vel_pub.publish(smoothed_cmd)
    
    def check_sensor_timeout(self):
        """Check if sensor data is stale"""
        if rospy.Time.now() - self.last_scan_time > self.scan_timeout:
            rospy.logwarn("Sensor timeout! Stopping robot.")
            stop_cmd = Twist()  # All zeros
            self.cmd_vel_pub.publish(stop_cmd)
            return True
        return False
    
    def run(self):
        """Main execution loop"""
        rate = rospy.Rate(20)  # Higher frequency for smoother control
        
        rospy.loginfo("Starting obstacle avoidance...")
        
        while not rospy.is_shutdown():
            # Safety check for sensor timeout
            self.check_sensor_timeout()
            rate.sleep()
        
        # Ensure robot stops when shutting down
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.loginfo("Obstacle avoidance node shutdown complete.")

if __name__ == '__main__':
    try:
        robot = OptimizedObstacleAvoidance()
        robot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Obstacle avoidance interrupted.")
    except Exception as e:
        rospy.logerr("Error in obstacle avoidance: %s", str(e))