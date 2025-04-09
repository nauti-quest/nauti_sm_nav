#!/usr/bin/python3
import time
from math import pi

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion, Transform
from mavros_msgs.srv import SetMode, CommandHome
from nauti_sm_nav.msg import Command, BoundingBox
from pid import PID

OBJECTS_OF_INTEREST = {"person", "traffic_cone", "belgian_gate", "umbrella", "airplane"}

def DEG2RAD(degs):
    return degs * (pi/180)
        
def RAD2DEG(rads):
    return rads * (180/pi)

def angle_diff(from_theta, to_theta):
    diff = to_theta - from_theta
    if (diff > pi):
        diff = diff - 2*pi
    
    if (diff < -pi):
        diff = diff + 2*pi

    return diff

class Controller(object):
    
    def __init__(self):
        self.rate = rospy.Rate(10) # 10hz
        self.cmd = rospy.Publisher('loco/command', Command, queue_size=1)
        self.current_angles = [0, 0 ,0]
        # self.set_mode('STABILIZE')
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/detections', BoundingBox, self.detections_callback)
        self.current_thrust = 0.0
        self.object_detected = False

        self.surge_pid = PID(kp=3, ki=0, deriv_prediction_dt=0.3, max_deriv_noise_gain=3) # surge
        self.yaw_pid = PID(kp=3, ki=0, deriv_prediction_dt=0.3, max_deriv_noise_gain=3) # yaw

        rospy.loginfo('Controller initialized')

    def set_mode(self, mode):
        self.mode = mode

        if mode == 'MANUAL':
            self.set_manual_mode()
        elif mode == 'STABILIZE':
            self.set_stabilized_mode()
        elif mode == 'DEPTH HOLD':
            self.set_depth_hold_mode()
        elif mode == 'POSITION HOLD':
            self.set_position_hold_mode()

    def set_manual_mode(self):
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rospy.wait_for_service('mavros/set_mode')
        set_mode(0, 'MANUAL')

    def set_stabilized_mode(self):
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rospy.wait_for_service('mavros/set_mode')
        set_mode(0, 'STABILIZE')

    def set_depth_hold_mode(self):
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rospy.wait_for_service('mavros/set_mode')
        set_mode(0, 'ALT_HOLD')

    def set_home_position(self):
        rospy.wait_for_service('/mavros/cmd/set_home')
        try:
            set_home = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
            response = set_home(current_gps=True, latitude=0, longitude=0, altitude=0)
            if response.success:
                rospy.loginfo("Home position set successfully")
            else:
                rospy.logerr("Failed to set home position")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def set_position_hold_mode(self):
        set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        rospy.wait_for_service('mavros/set_mode')
            
        try:
            response = set_mode(0, 'POSHOLD')
            if response.mode_sent:
                rospy.loginfo("Successfully set mode to POSHOLD")
            else:
                rospy.logerr("Failed to set mode to POSHOLD")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def detections_callback(self, msg):
        if msg is not None:
            self.object_detected = msg.class_name in OBJECTS_OF_INTEREST

    def imu_callback(self, data):
        orientation = data.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_angles = [DEG2RAD(roll), DEG2RAD(pitch), DEG2RAD(yaw)]

    def heave(self, duration=1, thrust=0.25):
        rospy.loginfo('Controller running heave')

        timeout = time.time() + duration
        msg = Command()
        msg.heave = thrust

        while not(time.time() > timeout):
            self.cmd.publish(msg)
            self.rate.sleep()
    
    # TODO support distance.
    def thrust(self, duration=1, thrust=0.25, distance=None):
        rospy.loginfo('Controller running thrust')

        timeout = time.time() + duration
        msg = Command()
        msg.throttle = thrust

        while not(time.time() > timeout) and not self.object_detected:
            self.cmd.publish(msg)
            self.rate.sleep()

    # Target angles are [R,P,Y].
    def to_orientation(self, target_angles = [0,0,0], duration=None, thrust=None, tolerance=25, constraints=[True, True, True]):
        roll_target = DEG2RAD(target_angles[0])
        pitch_target = DEG2RAD(target_angles[1])
        yaw_target = DEG2RAD(target_angles[2])

        constraints = [roll_target!=0, pitch_target!=0, yaw_target!=0]
        tolerance = DEG2RAD(tolerance)

        rospy.loginfo('Controller going to target angles [RPY-deg]: %f %f %f'%(roll_target, pitch_target, yaw_target))

        if duration == None: #Using angle target
            rospy.loginfo('No duration given, so using position estimate.')
            rospy.loginfo('Constraining R(%r) P(%r) Y(%r) by %d' % (constraints[0], constraints[1], constraints[2], tolerance))
            (roll_init, pitch_init, yaw_init) = self.current_angles
            (roll_d, pitch_d, yaw_d) = [angle_diff(roll_init, roll_target), angle_diff(pitch_init, pitch_target), angle_diff(yaw_init, yaw_target)]

            rospy.loginfo('Init angles: R=%f, P=%f, Y=%f'%(RAD2DEG(roll_init),RAD2DEG(pitch_init),RAD2DEG(yaw_init)))
            rospy.loginfo('Target angles: R=%f, P=%f, Y=%f'%(RAD2DEG(roll_target),RAD2DEG(pitch_target),RAD2DEG(yaw_target)))
            rospy.loginfo('Difference: R=%f, P=%f, Y=%f'%((RAD2DEG(roll_d),RAD2DEG(pitch_d),RAD2DEG(yaw_d))))
            
            (roll, pitch, yaw) = self.current_angles

            while ((not constraints[0]) or abs(roll_d) > tolerance) or ((not constraints[1]) or abs(pitch_d) > tolerance) or ((not constraints[2]) or abs(yaw_d) > tolerance):
                rospy.loginfo('-----STEP-----')
                rospy.loginfo('Current: R=%f, P=%f, Y=%f'%(RAD2DEG(roll),RAD2DEG(pitch),RAD2DEG(yaw)))
                rospy.loginfo('Target R=%f, P=%f, Y=%f'%(RAD2DEG(roll_target),RAD2DEG(pitch_target),RAD2DEG(yaw_target)))
                rospy.loginfo('Difference: R=%f, P=%f, Y=%f'%((RAD2DEG(roll_d),RAD2DEG(pitch_d),RAD2DEG(yaw_d))))
                
                msg = Command()

                msg.roll = (roll_target!=0) * thrust
                msg.pitch = (pitch_target!=0) * thrust
                msg.yaw = (yaw_target!=0) * thrust

                self.cmd.publish(msg)
                self.rate.sleep()
                (roll, pitch, yaw) = self.current_angles
                (roll_d, pitch_d, yaw_d) = [angle_diff(roll, roll_target), angle_diff(pitch, pitch_target), angle_diff(yaw, yaw_target
                )]

            # get current angle estimate from odometry, then loop over odom until close enough.

        else: # Using duration and thrust.
            rospy.loginfo('Given duration, overriding position estimate.')
            timeout = time.time() + duration
            msg = Command()

            msg.pitch = thrust
            msg.yaw = thrust * (-1 if target_angles[2] < 0 else 1)

            while not(time.time() > timeout) and not self.object_detected:
                self.cmd.publish(msg)
                self.rate.sleep()

    def thurst_and_orientation(self):
        pass

    def do_square(self, sideA_length=1, sideB_length=1, thrust=0.5, right=True):
        sideA_duration = sideA_length * 10
        sideB_duration = sideB_length * 10

        self.thrust(sideA_duration, thrust) # Leg 1
        self.to_orientation(target_angles=[0,0,90], duration=0.85, thrust=thrust)
        self.thrust(sideB_duration, thrust) # Leg 2
        self.to_orientation(target_angles=[0,0,90], duration=0.85, thrust=thrust)
        self.thrust(sideA_duration, thrust) # Leg 3
        self.to_orientation(target_angles=[0,0,90], duration=0.85, thrust=thrust)
        self.thrust(sideB_duration, thrust) # Leg 4
        self.to_orientation(target_angles=[0,0,90], duration=0.85, thrust=thrust)

    # def do_square(self, sideA_length=1, sideB_length=1, thrust=0.5, right=True):
    #     sideA_duration = sideA_length * 5
    #     sideB_duration = sideB_length * 5
        
    #     def execute_leg(duration, target_thrust, target_yaw):
    #         timeout = time.time() + duration
    #         while not(time.time() > timeout):
    #             current_time = time.time()
                
    #             # Calculate thrust error and control
    #             thrust_error = target_thrust - self.current_thrust  # You'll need to track current thrust
    #             self.surge_pid.update(thrust_error, current_time)
    #             thrust_control = self.surge_pid.control
                
    #             # Calculate yaw error and control
    #             yaw_error = angle_diff(self.current_angles[2], target_yaw)
    #             self.yaw_pid.update(yaw_error, current_time)
    #             yaw_control = self.yaw_pid.control
                
    #             # Create and publish command
    #             msg = Command()
    #             msg.throttle = max(min(thrust_control, 1.0), -1.0)  # Clamp between -1 and 1
    #             msg.yaw = max(min(yaw_control, 1.0), -1.0)  # Clamp between -1 and 1
    #             self.cmd.publish(msg)
    #             self.rate.sleep()

    #     current_yaw = self.current_angles[2]
    #     yaw_increment = pi/2 if right else -pi/2
    #     execute_leg(sideA_duration, thrust, current_yaw)
        
    #     current_yaw += yaw_increment
    #     execute_leg(sideB_duration, thrust, current_yaw)
        
    #     current_yaw += yaw_increment
    #     execute_leg(sideA_duration, thrust, current_yaw)
        
    #     current_yaw += yaw_increment
    #     execute_leg(sideB_duration, thrust, current_yaw)
        
    #     current_yaw += yaw_increment
    #     execute_leg(1.5, 0, current_yaw)

    def do_circle(self, duration=1000, radius=0, thrust=15):
        #TODO radius not yet supported.
        timeout = time.time() + duration
        msg = Command()
        msg.throttle = thrust
        while not (time.time() > 10):
            self.cmd.publish(msg)
            self.rate.sleep()
        msg.yaw = thrust / 45

        while not(time.time() > timeout):
            self.cmd.publish(msg)
            self.rate.sleep()

    def do_lawn_moving(self, thrust=20):
        self.thrust(15, thrust=thrust)
        self.to_orientation([0, 0, -90], duration=0.85, thrust=thrust)
        self.thrust(4, thrust=thrust)
        self.to_orientation([0, 0, -90], duration=0.85, thrust=thrust)
        self.thrust(15, thrust=thrust)
        self.to_orientation([0, 0, 90], duration=0.85, thrust=thrust)
        self.thrust(4, thrust=thrust)
        self.to_orientation([0, 0, 90], duration=0.85, thrust=thrust)

if __name__ == '__main__':
    try:
        controller = Controller()
        # controller.do_square()
        controller.do_lawn_moving()
    except rospy.ROSInterruptException:
        pass