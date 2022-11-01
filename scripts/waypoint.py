#!/usr/bin/env python

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler

class Waypoints():
    def __init__(self):
        rospy.init_node('waypoint')
        self.new_points = list()
        self.goal_cnt = 0
        try:
            self.points = rospy.get_param('waypoints/points')
        except KeyError:
            self.start_record_points = True
            self.end_record_points = False
            rospy.logerr("No points have provided, listening from /move_base_msgs instead")
            self.new_points = PoseStamped()
            self.sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.rviz_cb)
        
        if(self.start_record_points):
            #dont do anything until it finishes record the points needed

            wait = 'n'
            while wait == 'n':
                rospy.logdebug("Waiting for move_base_simple/goal topic to publish points")
                wait = input("Enter 'y' to stop recording waypoints")                
            
            self.end_record_points = True
            self.start_record_points = False
            #TODO: output the recorded points to yaml
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("move_base Action Server is not available")
            rospy.signal_shutdown("move_base Action server is not available")
            return
        rospy.logdebug("Connected to the move_base server")
        rospy.logdebug("Starting navigation to points...")
        self.movebase_client()
    
    def movebase_client():
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = self.new_points[0]
        
        
    def active_cb(self):
        rospy.logdebug("Robot currently navigating to the next destination")
    
    def feedback_cb(self):
        rospy.logdebug("Feedback is received")
    
    def done_cb(self, status, result):
        self.goal_cnt += 1
        if status == 2:
            #Received a cancel request
            rospy.logdebug("Cancel request fulfilled. Goal is fulfilled!")
        if status == 3:
            #A Goal is reached
            rospy.logdebug("A goal is reached!")
        if status == 4:
            #A Goal is aborted by the action server
            rospy.logdebug("Goal was aborted by the action server. Maybe the robot is stuck")
        if status == 5:
            #Goal pose is rejected by the action server
            rospy.logdebug("Goal pose was rejected, maybe the goal is not feasible")
        if status == 8:
            #Goal pose (thiscount) received a cancel before started executing, cancelled!
            rospy.logdebug("This goal received a cancel. Cancelled successfully!")
        else:
            #didnt know the status meaning
            rospy.logdebug("Not sure about the status of action server is giving back")
    
    def send_goal(self, goal):
        self.client

    def rviz_cb(self, data):
        if(self.end_record_points != True):
            temporary = PoseStamped()
            temporary = data
            self.new_points.append(temporary.pose.position)
            self.goal_cnt += 1
            rospy.logdebug("A Point is Added:")
            rospy.logdebug(data)
        else:
            rospy.logdebug("Getting points but currently not recording points")

if __name__ == '__main__':
    try:
        Waypoints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoints cancelled")