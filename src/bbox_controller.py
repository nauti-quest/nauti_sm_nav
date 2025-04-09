#!/usr/bin/python3

import rospy
from nauti_sm_nav.msg import Command, BoundingBox
from std_msgs.msg import Float32
from math import pi, sqrt, exp, log, tanh, cos, sin
from threading import Lock
import numpy as np

from pid import PID

"""
 Accept bbox as input (msg type BoundingBox)
 Issues (speed, yaw, pitch) commands for following it
 PID controller smooths the control outputs

  Launch the bbox_yaw_pitch_controller.launch file
  Tuned PID params are in data/bbox_yaw_pitch_ctr_pid_params.yaml
"""
# false positives avoidance - use probability
OBJECTS_OF_INTEREST = {"person", "traffic_cone", "belgian_gate", "umbrella", "airplane", "boat"}
THRESHOLD = 0.10

# Provides helper functions for BBox object
class BBoxFilter(object):
    def __init__(self, img_cols, img_rows, bbox_init_width, bbox_init_height):
        img_center_x = img_cols/2
        img_center_y = img_rows/2

        top_left_x = img_center_x - bbox_init_width/2.0
        top_left_y = img_center_y - bbox_init_height/2.0

        self.img_rows = img_rows
        self.img_cols = img_cols

        # state vector = [ normalized top_left_x,
        #                  normalized top_left_y,   ]
        #                  normalized bbox width, 
        #                  normalized bbox height, 
        #                  normalized pixel velocity of top_left_x,
        #                  normalized pixel velocity of top left y,
        #                  normalized pixel velocity of bbox width,
        #                  normalized pixel velocity of bbox height,
        #]    
        self.state = np.array([top_left_x / float(self.img_cols),
                               top_left_y / float(self.img_rows),
                               bbox_init_width / float(self.img_cols),
                               bbox_init_height / float(self.img_rows),
                               0., 0., 0., 0.])
        
        self.last_measurement = None
        
    def is_initialized(self):
        return (self.last_measurement is not None 
        and self.last_measurement.class_name in OBJECTS_OF_INTEREST 
        and self.last_measurement.class_prob >= THRESHOLD)

    def still_tracking(self, thr):
        return (rospy.Time.now() - self.last_measurement.header.stamp).to_sec() < thr
    
    def get_bbox(self):
        return self.state[0:4] * np.array([self.img_cols, self.img_rows, self.img_cols, self.img_rows])
    
    def update_estimate(self, measurement):
        self.last_measurement = measurement
        self.last_measurement.header.stamp = rospy.Time.now()
            
        z = np.array([self.last_measurement.top_left_x / self.img_cols,
                    self.last_measurement.top_left_y / self.img_rows,
                    self.last_measurement.width / self.img_cols,
                    self.last_measurement.height / self.img_rows]) 

        if (not self.is_initialized()):
            self.state[0:4] = z
            return
        
        self.state[0:4] = 0.*self.state[0:4] + 1.*z 
        
        

# The controller        
class BBoxReactiveController(object):
    def __init__(self):

        self.surge_pid = PID(kp=3, ki=0, deriv_prediction_dt=0.3, max_deriv_noise_gain=3) # surge
        self.heave_pid = PID(kp=5, ki=0, deriv_prediction_dt=0.6, max_deriv_noise_gain=3) # heave
        self.yaw_pid = PID(kp=5, ki=0, deriv_prediction_dt=0.6, max_deriv_noise_gain=3) # yaw
        self.roll_pid = PID(kp=3, ki=0, deriv_prediction_dt=0.3, max_deriv_noise_gain=3) # roll
        self.sway_pid = PID(kp=3, ki=0, deriv_prediction_dt=0.3, max_deriv_noise_gain=3) # sway
        self.params_map = {}
        self.set_pid_params()

        self.current_state = None
        self.bbox_filter = None     

        self.current_state_mutex = Lock()        
        self.current_observation_mutex = Lock()
        self.current_pid_mutex = Lock()
        
        self.rate = 20 

        print ("Waiting for /detections to come up")
        rospy.wait_for_message('/detections', BoundingBox)
        print ("/detections has come up")
        
        self.observation_sub = rospy.Subscriber("/detections", BoundingBox, self.observation_callback, queue_size=3)
        self.rpy_pub = rospy.Publisher('/loco/command', Command, queue_size=3)
        self.cmd_msg = Command()
	
        
    def observation_callback(self, msg):
        self.current_observation_mutex.acquire()
        self.current_observation = None

        if msg.class_name in OBJECTS_OF_INTEREST: # only assign the current observation if there is something to observe
            self.current_observation = msg

        if self.bbox_filter is None and self.current_observation:
            self.bbox_filter = BBoxFilter(self.current_observation.image_width, self.current_observation.image_height,
                                          self.current_observation.image_width/2.0, self.current_observation.image_height/2.0)

        if self.current_observation:
	    #print ('Got a measurement, updating now')
            self.bbox_filter.update_estimate(self.current_observation)

        self.current_observation_mutex.release()
            
    def set_pid_params(self):
        self.params_map['flat_vel_kp'] = rospy.get_param('~flat_vel_kp')
        self.params_map['flat_vel_ki'] = rospy.get_param('~flat_vel_ki')
        self.params_map['flat_vel_deriv_prediction_dt'] = rospy.get_param('~flat_vel_deriv_prediction_dt')
        self.params_map['flat_vel_max_deriv_noise_gain'] = rospy.get_param('~flat_vel_max_deriv_noise_gain')
	#print (self.params_map['flat_vel_kp'], self.params_map['flat_vel_ki'], self.params_map['flat_vel_deriv_prediction_dt'])

        self.params_map['flat_yaw_kp'] = rospy.get_param('~flat_yaw_kp')
        self.params_map['flat_yaw_ki'] = rospy.get_param('~flat_yaw_ki')
        self.params_map['flat_yaw_deriv_prediction_dt'] = rospy.get_param('~flat_yaw_deriv_prediction_dt')
        self.params_map['flat_yaw_max_deriv_noise_gain'] = rospy.get_param('~flat_yaw_max_deriv_noise_gain')
	#print (self.params_map['flat_yaw_kp'], self.params_map['flat_yaw_ki'], self.params_map['flat_yaw_deriv_prediction_dt'])
	

        self.params_map['flat_sway_kp'] = rospy.get_param('~flat_sway_kp')
        self.params_map['flat_sway_ki'] = rospy.get_param('~flat_sway_ki')
        self.params_map['flat_sway_deriv_prediction_dt'] = rospy.get_param('~flat_sway_deriv_prediction_dt')
        self.params_map['flat_sway_max_deriv_noise_gain'] = rospy.get_param('~flat_sway_max_deriv_noise_gain')

        self.params_map['magnify_speed'] = rospy.get_param('~magnify_speed')
        self.params_map['deadzone_abs_vel_error'] = rospy.get_param('~deadzone_abs_vel_error')
        self.params_map['deadzone_abs_yaw_error'] = rospy.get_param('~deadzone_abs_yaw_error')
        self.params_map['deadzone_abs_heave_error'] = rospy.get_param('~deadzone_abs_heave_error')
        self.params_map['target_bbox_image_ratio'] = rospy.get_param('~target_bbox_image_ratio')
        self.params_map['sec_before_giving_up'] = rospy.get_param('~sec_before_giving_up')

        self.surge_pid.set_params(self.params_map['flat_vel_kp'], self.params_map['flat_vel_ki'], 
			       self.params_map['flat_vel_deriv_prediction_dt'], self.params_map['flat_vel_max_deriv_noise_gain'])
        self.yaw_pid.set_params(self.params_map['flat_yaw_kp'], self.params_map['flat_yaw_ki'], 
			       self.params_map['flat_yaw_deriv_prediction_dt'], self.params_map['flat_yaw_max_deriv_noise_gain'])
        self.sway_pid.set_params(self.params_map['flat_sway_kp'], self.params_map['flat_sway_ki'], 
			       self.params_map['flat_sway_deriv_prediction_dt'], self.params_map['flat_sway_max_deriv_noise_gain'])
        self.heave_pid.set_params(self.params_map['flat_vel_kp'], self.params_map['flat_vel_ki'], 
			       self.params_map['flat_vel_deriv_prediction_dt'], self.params_map['flat_vel_max_deriv_noise_gain'])
        self.roll_pid.set_params(self.params_map['flat_yaw_kp'], self.params_map['flat_yaw_ki'], 
			       self.params_map['flat_yaw_deriv_prediction_dt'], self.params_map['flat_yaw_max_deriv_noise_gain'])     
   
            
    def compute_errors_from_estimate(self): 
        bbox = self.bbox_filter.get_bbox()
        top_left_x, top_left_y, bbox_width, bbox_height = tuple(bbox)
        
        bbox_center_x = top_left_x + bbox_width/2.0
        bbox_center_y = top_left_y + bbox_height/2.0

        image_center_x = self.bbox_filter.img_cols/2.0
        image_center_y = self.bbox_filter.img_rows/2.0
        
        error_cols = (bbox_center_x - image_center_x) / float(self.bbox_filter.img_cols)  # [-1,1]
        error_rows = (bbox_center_y - image_center_y) / float(self.bbox_filter.img_rows)  # [-1,1]

        bbox_area = bbox_width * bbox_height
        image_area = self.bbox_filter.img_cols * self.bbox_filter.img_rows

        error_bbox_size = self.params_map['target_bbox_image_ratio'] * (1.0 - bbox_area/float(image_area)) 
        print(error_bbox_size)
        # error_bbox_size = max(0.0, error_bbox_size) # [0, target_bbox_im_ratio] \propoto distance
        
        error_forward = error_bbox_size
        return (error_forward, error_cols, error_rows)



    def _clip(self, value, min_value, max_value):
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        else:
            return value


        
    def _acquire_all_mutexes(self):
        self.current_observation_mutex.acquire()
        self.current_pid_mutex.acquire()

    def _release_all_mutexes(self):
        self.current_pid_mutex.release()
        self.current_observation_mutex.release()


        
    def compute_control(self):
        self._acquire_all_mutexes()
        
        now = rospy.Time.now()
        bbox_filter_is_active = (self.bbox_filter is not None and self.bbox_filter.is_initialized()
				     and self.bbox_filter.still_tracking(self.params_map['sec_before_giving_up']))
        

        if bbox_filter_is_active:
            ss, yy, pp, rr, hh = 0, 0, 0, 0, 0
            error_forward, error_yaw, error_heave = self.compute_errors_from_estimate()
            #print (error_forward, error_yaw,  error_pitch)
    
            self.surge_pid.update(error_forward, now.to_sec())
            self.yaw_pid.update(error_yaw, now.to_sec())
            self.heave_pid.update(error_heave, now.to_sec())

            if self.surge_pid.is_initialized(): # forward pseudospeed
                ss = self._clip(self.surge_pid.control-self.params_map['target_bbox_image_ratio'], -1, 1)  
                if abs(ss) <= self.params_map['deadzone_abs_vel_error']:
                    ss = 0.0 
                else: 
                    ss = self._clip(self.params_map['magnify_speed']*ss, -1, 1)  

            if self.yaw_pid.is_initialized(): # yaw pseudospeed
                yy = self._clip(self.yaw_pid.control, -1, 1)
                if abs(yy) <= self.params_map['deadzone_abs_yaw_error']:
                    yy = 0.0           

            if self.heave_pid.is_initialized(): # pitch pseudospeed         
                hh = self._clip(self.heave_pid.control, -1, 1) 
                if abs(hh) <= self.params_map['deadzone_abs_heave_error']:
                    hh = 0.0

            print ('V, yaw, heave : ', (ss, yy,  hh))
            self.set_vyprh_cmd(ss, yy, pp, rr, -(hh))

        else:
            print ('Target out of sight or statioanry')
            self.set_vyprh_cmd(0, 0, 0, 0, 0)

        self._release_all_mutexes()        
        return 

	

    def set_vyprh_cmd(self, ss, yy, pp, rr, hh):
        self.cmd_msg.throttle = 0
        self.cmd_msg.yaw = 0
        if ss < 0:
            self.cmd_msg.throttle = ss - 0.3
        elif ss > 0:
            self.cmd_msg.throttle = ss + 0.3
        if yy > 0:
            self.cmd_msg.yaw = yy + 0.4
        elif yy < 0:
            self.cmd_msg.yaw = yy - 0.4
        self.cmd_msg.pitch = pp
        self.cmd_msg.roll = rr
        self.cmd_msg.heave = hh
        


    def publish_control(self):
        #print ('publishing ', self.cmd_msg)
        self.rpy_pub.publish(self.cmd_msg)


        
if __name__ == "__main__":
    bbrc = BBoxReactiveController()
    
    rate = rospy.Rate(bbrc.rate)
    while not rospy.is_shutdown(): 
        bbrc.compute_control()
        bbrc.publish_control()
        rate.sleep()
