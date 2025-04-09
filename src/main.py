#!/usr/bin/python3
from bbox_controller import BBoxReactiveController
from controller import Controller, OBJECTS_OF_INTEREST
from state import State
from nauti_sm_nav.msg import BoundingBox
import rospy
import time

class LawnMoving(State):
    def __init__(self):
        self.object_detected = False
        self.detection_sub = rospy.Subscriber("/detections", BoundingBox, self.detection_callback)
        self.controller = Controller()

    def detection_callback(self, msg):
        if msg is not None:
            self.object_detected = msg.class_name in OBJECTS_OF_INTEREST

    def execute(self):
        rospy.loginfo("Executing lawn moving maneuver.")
        self.controller.do_lawn_moving(thrust=10)
        rospy.sleep(0.5)  # simulate action delay

    def check_transition(self):
        if self.object_detected:
            rospy.loginfo("Object detected. Switching to Bounding Box Navigation.")
            return BoundingBoxNavigation()
        return self

class BoundingBoxNavigation(State):
    def __init__(self):
        self.object_detected = True
        # Subscribe to the same topic to get updated detection status.
        self.detection_sub = rospy.Subscriber("/detections", BoundingBox, self.detection_callback)
        self.bbrc = BBoxReactiveController()
        self.last_detected = time.time() # 1 second grace time
        self.timeout = 10  # seconds before timeout

    def detection_callback(self, msg):
        if msg is not None:
           self.object_detected = msg.class_name in OBJECTS_OF_INTEREST
           self.last_detected = time.time()

    def execute(self):
        rospy.loginfo("Navigating towards the detected object.")
        self.bbrc.compute_control()
        self.bbrc.publish_control()
        rospy.sleep(0.5)  # simulate action delay

    def check_transition(self):
        # If the object is lost, revert back to LawnMoving.
        if not self.object_detected and time.time() - self.last_detected > self.timeout:
            rospy.loginfo("Object lost. Reverting to Lawn Moving maneuver.")
            return LawnMoving()
        return self


class StateMachine(object):
    def __init__(self):
        # Start with lawn moving 
        self.state = LawnMoving()

    def run(self):
        rate = rospy.Rate(10)  # adjust rate as needed
        while not rospy.is_shutdown():
            self.state.execute()
            # Check for transitions
            self.state = self.state.check_transition()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("sm_nav_node")
    sm = StateMachine()
    try:
        time.sleep(5) # sleeping for 5 seconds to switch to window with gazebo and rqt image view
        sm.run()
    except rospy.ROSInterruptException:
        pass