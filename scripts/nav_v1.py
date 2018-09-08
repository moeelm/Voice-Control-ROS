#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction

class Motion(object):
    def __init__(self): 
        
        rospy.on_shutdown(self.shutdown)
        
        
        self.pub_pos = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        
        rospy.sleep(3)
        rospy.loginfo("Wait for the action server to come up")
        text2_topic = rospy.get_param('/text2_topic', '/intent')
        rospy.Subscriber(text2_topic, String, self.callback, queue_size=10)
        
    
    def callback(self, ros_data):
        # Dictionary with various spatial locations, change according to position
        self.my_intent = {'microwave':{'position':{'x':2.00, 'y':-5.000, 'z':0.000}},
            'bathroom':{'position':{'x':0.49, 'y':-7.15, 'z':-0.002}}
            } 
        self.word_received = ros_data.data
        for key in sorted(self.my_intent.keys()):
            if self.word_received == key:
                self.myGoal = self.my_intent[key]['position']
    
    def start(self):
        rospy.loginfo("Starting Navigation")
       
        
    def go_to(self):
        rospy.sleep(5)  
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.z = self.myGoal['z']
        goal.pose.position.x = self.myGoal['x']
        goal.pose.position.y = self.myGoal['y']
        goal.pose.orientation.w = 1.0
        self.pub_pos.publish(goal)
        rospy.loginfo("Sending goal")
        rospy.signal_shutdown("Restarting nav node")
        rospy.sleep(5)

    
    def shutdown(self):
        rospy.loginfo("Stop")
        self.closed = True
        exit()
    
if __name__ == '__main__':
    try:
        rospy.init_node("nav_v1")
        navigator = Motion()
        navigator.start()
        success = navigator.go_to()
        if success:
            rospy.loginfo("Reached desired destination")
        else:
            rospy.loginfo("Sorry, I was not able to reach the destination. Try again")
    except KeyboardInterrupt:
        navigator.shutdown()
    
      
        
            
        
        
        
        
        
