#!/usr/bin/env python
# Author: Rui Zhou 
# email: rzho774@aucklanduni.ac.nz

import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge

class hand_position:
    def __init__(self):
        self.bridge = CvBridge()
        self.joint_sub = rospy.Subscriber("/body_tracking_data", MarkerArray, self.joint_callback)
        self.rhx, self.rhy, self.rhz = None, None, None
        self.pelvx, self.pelvy, self.pelvz = None, None, None
        self.hand_in_boundary = False
        self.current_distance = None

    def joint_callback(self, data):
        try:
            if data.markers:
                for marker in data.markers:
                    joint_id = marker.id % 100
                    if joint_id == 0:  # Pelvis
                        self.pelvx, self.pelvy, self.pelvz = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z
                    elif joint_id == 15:  # Right hand
                        self.rhx, self.rhy, self.rhz = self.transform_palm_coords(
                            marker.pose.position.x, 
                            marker.pose.position.y, 
                            marker.pose.position.z
                        )
                        self.rhx -= 0.02
                        self.rhz += 0.32
                        self.check_hand_boundary()
                
                self.current_distance = np.sqrt((self.pelvx - self.rhx)**2 + (self.pelvy - self.rhy)**2 + (self.pelvz - self.rhz)**2)
            else:
                self.current_distance = 0
        except:
            self.current_distance = 0

    def check_hand_boundary(self):
        if ((self.rhx > 0.24) and (self.rhx < 0.88) and (self.rhy > -0.48) and (self.rhy < 0.6) and (self.rhz > 0.05) and (self.rhz < 0.6)):
            self.hand_in_boundary = True
        else:
            self.hand_in_boundary = False



