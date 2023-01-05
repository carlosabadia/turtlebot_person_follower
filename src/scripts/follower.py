#!/usr/bin/env python3
import rospy
import smach
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from person_follower.msg import center


# Constants
MIN_DISTANCE_TO_OBJECT = 0.6  # Minimum distance to an object to consider it detected to stop
WIDTH = 320  # Width of the camera image
OFFSET = 30  # Allowed offset from the center of the image
MINIMUM_SPEED = 0.6  # Minimum speed of the robot
MAXIMUM_SPEED = 2  # Maximum speed of the robot

class LookForPerson(smach.State):
    """A state for looking for a person.

    This state subscribes to the /center topic to receive messages of type
    'center', which provide information about the person that the robot is
    following, including their name. The state has two outcomes: 'found_person'
    and 'not_found'. The state will return 'found_person' if it receives a
    message on the /center topic with a non-empty 'person' field, and
    'not_found' otherwise.
    """
    def __init__(self):
        # Initialize the state with two possible outcomes: found_person and not_found
        smach.State.__init__(self, outcomes=['found_person', 'not_found'])

        # Subscribe to the /center topic to receive messages of type center
        rospy.Subscriber("/center", center, self.center_cb, queue_size=10)
        # Create a publisher to the /cmd_vel topic with a queue size of 10
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Create an empty Twist message to be used for publishing later
        self.speed = Twist()
        # Set the default name to "Unknown"
        self.name = "Unknown"

    def center_cb(self, data):
        """Callback for the /center topic.

        This callback updates the 'name' attribute based on the 'person' field
        of the received message.
        """
        self.name = data.person
    
    def execute(self, userdata):
        """Execute the state.

        This method searches for a person and returns the appropriate outcome.
        """
        # Check if the name is not "Unknown"
        if self.name != "Unknown":
            # If the name is known, return found_person
            return 'found_person'
        else:
            # If the name is unknown, set the angular velocity of the Twist message
            # and publish it to the /cmd_vel topic
            self.speed.angular.z = 0.2
            self.pub.publish(self.speed)
            # Return not_found since the person has not been found yet
            return 'not_found'


        
class FollowPerson(smach.State):
    """A state for following a person.

    This state subscribes to two topics: /center and /scan. The /center topic
    provides information about the person that the robot is following, including
    their name and the x-coordinate of their center. The /scan topic provides
    laser scan data from the robot's laser rangefinder.

    The state has three outcomes: 'following', 'person_close', and
    'look_for_person'. The state will return 'following' if it is successfully
    following a person, 'person_close' if the person is too close to the robot or found an obstacle,
    and 'look_for_person' if it cannot see the person to follow.
    """
    def __init__(self):
        # Initialize the state with the given outcomes.
        smach.State.__init__(self, outcomes=['following', 'person_close', 'look_for_person'])

        # Initialize instance variables.
        self.name = "Unknown"
        self.person_center_x = 0
        self.min_total = 0
        self.min_zone_left = 0
        self.min_zone_right = 0
        self.speed = Twist()

        # Subscribe to the /center and /scan topics.
        rospy.Subscriber("/center", center, self.center_cb, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_cb, queue_size=10)

        # Initialize the publisher for the /cmd_vel topic.
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def center_cb(self, data):
        """Callback for the /center topic.

        This callback updates the name and person_center_x instance variables
        based on the data received on the /center topic.
        """
        self.name = data.person
        self.person_center_x = data.x

    def laser_cb(self, data):
        """Callback for the /scan topic.

        This callback updates the min_zone_left, min_zone_right, and min_total
        instance variables based on the data received on the /scan topic.
        """
        self.min_zone_left = min(data.ranges[0:15])
        self.min_zone_right = min(data.ranges[345:360])
        self.min_total = min(self.min_zone_right, self.min_zone_left)

    def execute(self, userdata):
        """Execute the state.

        This method controls the robot's movement and returns the appropriate
        outcome.
        """
        # Set the linear velocity of the robot based on the minimum distance
        # to an object as reported by the laser rangefinder.
        if self.min_total < MIN_DISTANCE_TO_OBJECT:
            self.speed.linear.x = 0
        else:
            speed_map = (self.min_total - MINIMUM_SPEED) / (MAXIMUM_SPEED - MINIMUM_SPEED)
            self.speed.linear.x = speed_map
        # Set the angular velocity of the robot based on the position of the
        # person relative to the center of the camera image.
        if self.person_center_x < (WIDTH - OFFSET):
            self.speed.angular.z = 0.2
        elif self.person_center_x > (WIDTH + OFFSET):
            self.speed.angular.z = -0.2
        else:
            self.speed.angular.z = 0
        
        # Publish the Twist message to control the robot's movement.
        self.pub.publish(self.speed)
        
        # Determine and return the appropriate outcome.
        if self.min_total < MIN_DISTANCE_TO_OBJECT:
            return 'person_close'
        elif self.name == "Unknown":
            return 'look_for_person'
        else:
            return 'following'

       

        
class PersonClose(smach.State):
    """A state for when the person is too close to the robot.

    This state subscribes to the /scan topic to receive laser scan data. The
    state has three outcomes: 'not_found', 'look_for_person', and
    'person_close'. The state will return 'not_found' if it cannot see a person,
    'look_for_person' if the person is no longer too close to the robot, and
    'person_close' if the person is still too close to the robot.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['not_found', 'look_for_person', 'person_close'])

        # Initialize instance variables.
        self.name = "Unknown"
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_cb, queue_size=10)
        self.speed = Twist()
        
        # Initialize any necessary variables here

    def center_cb(self, data):
        """Callback for the /center topic.

        This callback updates the 'name' attribute based on the 'person' field
        of the received message.
        """
        self.name = data.person

    def laser_cb(self, data):
        """Callback for the /scan topic.

        This callback updates the min_zone_left, min_zone_right, and min_total
        instance variables based on the data received on the /scan topic.
        """
        self.min_zone_left = min(data.ranges[0:15])
        self.min_zone_right = min(data.ranges[345:360])
        self.min_total = min(self.min_zone_right, self.min_zone_left)

    def execute(self, userdata):
        """Execute the state.

        This method stops the robot's movement and returns the appropriate
        outcome.
        """
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
    """Main function for the follow person state machine.

    This function creates a state machine and adds three states to it:
    'LOOK_FOR_PERSON', 'FOLLOW_PERSON', and 'PERSON_CLOSE'. The state machine
    has a single outcome, 'success', and transitions between states based on the
    outcomes returned by the states.
    """
    # Initialize the ROS node
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
    
    # Execute the state machine without logging state change info
    outcome = sm.execute()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


