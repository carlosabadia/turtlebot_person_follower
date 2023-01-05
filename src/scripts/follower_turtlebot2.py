#!/usr/bin/env python3
import rospy
import smach
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from person_follower.msg import center

# Set the logger's log level to ERROR


# Constants
MIN_DISTANCE_TO_OBJECT = 0.6  # Minimum distance to an object to consider it detected
WIDTH = 320  # Width of the camera image
OFFSET = 30  # Allowed offset from the center of the image
MINIMUM_SPEED = 0.6  # Minimum speed of the robot
MAXIMUM_SPEED = 2  # Maximum speed of the robot

class LookForPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found_person', 'not_found'])

        rospy.Subscriber("/center", center, self.center_cb, queue_size=10)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.speed = Twist()
        self.name = "Unknown"

    def center_cb(self, data):
        self.name = data.person
    
        
    def execute(self, userdata):
        # Look for a person using the laser scanner and camera data

        if self.name != "Unknown":
            return 'found_person'
        else:
            self.speed.angular.z = 0.2
            self.pub.publish(self.speed)
            return 'not_found'
        
class FollowPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['following', 'person_close', 'look_for_person'])
        
        # Initialize any necessary variables here
        
        self.name = "Unknown"
        self.person_center_x = 0
        self.min_total = 0
        self.min_zone_left = 0
        self.min_zone_right = 0
        self.speed = Twist()
        rospy.Subscriber("/center", center, self.center_cb, queue_size=10)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_cb, queue_size=10)

    def center_cb(self, data):

        self.name = data.person
        self.person_center_x = data.x

    def laser_cb(self, data):
        self.min_total = min(data.ranges[320:639])
        
    
    def execute(self, userdata):
        
        if self.min_total< MIN_DISTANCE_TO_OBJECT:
            self.speed.linear.x = 0
        else: 
            speed_map = (self.min_total-MINIMUM_SPEED) / (MAXIMUM_SPEED-MINIMUM_SPEED)
            self.speed.linear.x = speed_map

        if self.person_center_x < (WIDTH - OFFSET):
            self.speed.angular.z = 0.2
        elif self.person_center_x > (WIDTH + OFFSET):
            self.speed.angular.z = -0.2
        else:
            self.speed.angular.z = 0
        
        self.pub.publish(self.speed)
        
        if self.min_total < MIN_DISTANCE_TO_OBJECT:
            return 'person_close'
        elif self.name == "Unknown":
            return 'look_for_person'
        else:
            return 'following'
        
class PersonClose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_found', 'look_for_person', 'person_close'])

        self.name = "Unknown"
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_cb, queue_size=10)
        self.speed = Twist()
        
        # Initialize any necessary variables here
    def center_cb(self, data):
        self.name = data.person

    def laser_cb(self, data):
        self.min_zone_left = min(data.ranges[0:15])
        self.min_zone_right = min(data.ranges[345:360])
        self.min_total = min(self.min_zone_right, self.min_zone_left)

    def execute(self, userdata):
        # Change the speed of the robot to 0
        self.speed.linear.x = 0
        self.speed.angular.z = 0
        self.pub.publish(self.speed)
        if self.name == "Unknown":
            return 'not_found'
        elif self.min_total < MIN_DISTANCE_TO_OBJECT:
            return 'person_close'
        else:
            return 'look_for_person'

def main():
    rospy.init_node('follow_person_smach')
    
    # Create the state machine
    sm = smach.StateMachine(outcomes=['success'])
    
    # Add the three states to the state machine
    with sm:
        smach.StateMachine.add('LOOK_FOR_PERSON', LookForPerson(), 
                               transitions={'found_person':'FOLLOW_PERSON', 
                                            'not_found':'LOOK_FOR_PERSON'})
        smach.StateMachine.add('FOLLOW_PERSON', FollowPerson(), 
                               transitions={'following':'FOLLOW_PERSON', 
                                            'person_close':'PERSON_CLOSE',
                                            'look_for_person':'LOOK_FOR_PERSON'})
        smach.StateMachine.add('PERSON_CLOSE', PersonClose(), 
                               transitions={'person_close':'PERSON_CLOSE',
                                            'look_for_person':'LOOK_FOR_PERSON',
                                            'not_found':'LOOK_FOR_PERSON'})
    
    # Execute the state machine wihout the logging state change info
    outcome = sm.execute()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

