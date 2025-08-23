#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

def send_goal(client, x, y, yaw=0.0):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # simple facing forward
    rospy.loginfo("Sending goal: (%.2f, %.2f)" % (x, y))
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_state()
    if result == GoalStatus.SUCCEEDED:
        rospy.loginfo("Reached goal!")
    else:
        rospy.logwarn("Failed to reach goal!")
    return result

if __name__ == "__main__":
    rospy.init_node("multi_goal_nav")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base!")

    # Define a list of goals (x, y)
    goals = [(1.0, 1.0), (2.0, -1.0), (0.0, -2.0), (-1.5, 0.5)]

    for g in goals:
        send_goal(client, g[0], g[1])

    rospy.loginfo("All goals completed!")
