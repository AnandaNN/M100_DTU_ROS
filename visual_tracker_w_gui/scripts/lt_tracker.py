#! /usr/bin/env python2
import numpy as np
import cv2 as cv2
import math
import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

C_MID = (640, 360)

class Target_tracker():
    def __init__(self):
        
        #for converting to openCV
        self.bridge = CvBridge()

        #for keeping track of the target
        self.previous_target = (None, None)
        self.new_target = (None, None)

        #the different frames
        self.frame = None

        self.no_change = 0

        #for the feature detector
        self.keypoints_previous_frame = None
        self.descriptors_previous_frame = None

        self.focal_length_x = 1068.0
        self.focal_length_y = 1072.0
        self.distance_to_wall = None
        self.wall_angle = None
        self.distance_error = Point()

        self.first_flag = True

        self.hoz_fov = np.arctan(C_MID[0] / self.focal_length_x)

        # for csv
        self.matches_array = []
        self.matches_used_array = [] 
        
        # publisher
        self.target_pub = rospy.Publisher('/target', Twist, queue_size=1)
        self.distance_error_pub = rospy.Publisher('/distance_error', Point, queue_size=1)

        # subscriber
        self.image_sub = rospy.Subscriber("/camera/image_decompressed",Image,self.read_frame)
        #self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.read_frame)
        self.gui_target_sub = rospy.Subscriber("/gui_target", Point, self.read_gui_target)
        self.distance_sub = rospy.Subscriber("/dtu_controller/current_frame_pose", Twist, self.update_distance)
        
        print('Target tracking initialised')

    def read_frame(self, data):
        ## converting image from drone to opencv format
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.first_flag = False

        except CvBridgeError as e:
            print(e)

    def read_gui_target(self, data):
        print("Target from GUI read")
        #if(self.new_target[0] == None):
        self.new_target = (data.x, data.y)
        #if(self.previous_target[0] == None):
        self.previous_target = (data.x, data.y)

    def find_matches(self):
        ## finds usable matches from previous and current frame, as well as update the csv file
        ## usable matches: excluding those within a radius of the laser dots and excluding those far away from target point
        orb = cv2.ORB_create()
        keypoints_frame, descriptors_frame = orb.detectAndCompute(self.frame, None)

        if( self.keypoints_previous_frame == None ):
            self.keypoints_previous_frame = keypoints_frame
            self.descriptors_previous_frame = descriptors_frame

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(self.descriptors_previous_frame, descriptors_frame)
        matches = sorted(matches, key=lambda x: x.distance)

        coordinates_best_matches_previous_frame = []
        coordinates_best_matches_frame = []

        number_of_matches_to_use = int(len(matches))

        radi = math.sqrt((self.previous_target[0]-C_MID[0])**2 + (self.previous_target[1]-C_MID[1])**2)
        factor = 100
        if radi < factor:
            radi = factor
        elif radi > 476 - factor:
            radi = 476 - factor

        for mat in matches[:number_of_matches_to_use]:
            previous_frame_idx = mat.queryIdx
            frame_idx = mat.trainIdx

            (x1, y1) = self.keypoints_previous_frame[previous_frame_idx].pt
            (x2, y2) = keypoints_frame[frame_idx].pt
            if math.sqrt((x1-C_MID[0])**2 + (y1-C_MID[1])**2) >= radi -factor and math.sqrt((x1-C_MID[0])**2 + (y1-C_MID[1])**2) <= radi + factor:
                coordinates_best_matches_previous_frame.append((x1, y1))
                coordinates_best_matches_frame.append((x2, y2))
        self.matches_array.append(len(matches))
        self.matches_used_array.append(len(coordinates_best_matches_frame))

        self.keypoints_previous_frame = keypoints_frame
        self.descriptors_previous_frame = descriptors_frame

        return (coordinates_best_matches_previous_frame, coordinates_best_matches_frame)

    def find_new_target(self):
        ## computes current target point based of coordinates of matches 

        (coordinates_best_matches_previous_frame, coordinates_best_matches_frame) = self.find_matches()

        prev_dist_to_target = []
        for i in range(0,len(coordinates_best_matches_frame)):
            prev_dist_to_target.append( (coordinates_best_matches_previous_frame[i][0]-self.previous_target[0], coordinates_best_matches_previous_frame[i][1]-self.previous_target[1]) )

        new_target_x = []
        new_target_y = []
        error_margin = 20
        for i in range(0, len(prev_dist_to_target)-1):
            if prev_dist_to_target[i][0] != prev_dist_to_target[i+1][0] and prev_dist_to_target[i][1] != prev_dist_to_target[i+1][1]:
               
                a_x = (coordinates_best_matches_frame[i+1][0]-coordinates_best_matches_frame[i][0]) / (prev_dist_to_target[i][0]- prev_dist_to_target[i+1][0])
                a_y = (coordinates_best_matches_frame[i+1][1]-coordinates_best_matches_frame[i][1]) / (prev_dist_to_target[i][1]- prev_dist_to_target[i+1][1])

                potentiel_new_x = prev_dist_to_target[i][0]*a_x + coordinates_best_matches_frame[i][0]
                potentiel_new_y = prev_dist_to_target[i][1]*a_y + coordinates_best_matches_frame[i][1]
                
                if potentiel_new_x-self.previous_target[0] < error_margin and potentiel_new_x-self.previous_target[0] > -error_margin and potentiel_new_y-self.previous_target[1] < error_margin and potentiel_new_y-self.previous_target[1] > -error_margin:
                    new_target_x.append( potentiel_new_x )
                    new_target_y.append( potentiel_new_y )
                    
        if len(new_target_y) != 0 and len(new_target_x) != 0 :
            self.new_target = (sum(new_target_x)/len(new_target_x), sum(new_target_y)/len(new_target_y))
            self.no_change = 1
        else:
            self.new_target = self.previous_target
            # print("old value used")
            self.no_change = 0
        
        self.previous_target = self.new_target

    def calculate_error(self):
        if self.distance_to_wall != None and self.new_target[0] != None:
            #self.distance_error.x = self.distance_to_wall
            #y_offset = (C_MID[0]) * np.sin(self.wall_angle)
            #self.distance_error.y = -((self.new_target[0]-y_offset) - C_MID[0])/self.focal_length_x * self.distance_to_wall
            #self.distance_error.z = -(self.new_target[1] - C_MID[1])/self.focal_length_y * self.distance_to_wall

            theta = self.wall_angle - (self.hoz_fov) * float(self.new_target[0] - C_MID[0])/float(C_MID[0])

            self.distance_error.y = self.distance_to_wall * np.sin(theta) # - self.distance_to_wall * np.sin(self.wall_angle)
            self.distance_error.z = -(self.new_target[1] - C_MID[1])/self.focal_length_y * self.distance_to_wall

    # Publish the target for the GUI to read for visualization
    def publish_new_target(self):
        if self.new_target[0] != None:
            p = Twist()
            p.linear.x = float(self.new_target[0]) # Current target position in image
            p.linear.y = float(self.new_target[1]) # Current target position in image
            p.linear.z = self.no_change            # Tell if target was succesfully tracked
            
            p.angular.x = float(0) # Send the current size of the bounding box
            p.angular.y = float(0) # Send the current size of the bounding box
            p.angular.z = 0 # Tell if bb is used

            self.target_pub.publish(p)
        
            # If distance errors are calculateable and tracking is succesfull publish
            if self.distance_to_wall != None and self.no_change:
                self.distance_error_pub.publish(self.distance_error)

    # def publish_new_target(self):
    #     if self.new_target[0] != None:
    #         # print("sending target")
    #         p = Point()
    #         p.x = float(self.new_target[0])
    #         p.y = float(self.new_target[1])
    #         p.z = self.no_change

    #         self.target_pub.publish(p)
    #     if self.distance_to_wall != None and self.no_change:
    #         self.distance_error_pub.publish(self.distance_error)

    def update_distance(self, data):
        self.distance_to_wall = data.linear.x
        self.wall_angle = data.angular.z

    def run(self):
        while not rospy.is_shutdown():

            if self.new_target[0] != None:
                if(not self.first_flag):
                    self.find_new_target()
                    self.calculate_error()
                    self.publish_new_target()

            rospy.Rate(30).sleep()


if __name__ == '__main__':
    rospy.init_node("target_node", anonymous=True)
    my_tracker = Target_tracker()
    my_tracker.run()




