#!/usr/bin/env python2

# GUI related
from Tkinter import Tk, Label, Button, Frame, Entry
from PIL import Image
from PIL import ImageTk

# ROS Related
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image as SensorImage
from sensor_msgs.msg import BatteryState
from std_msgs.msg import UInt8
from cv_bridge import CvBridge, CvBridgeError

class DroneGUI:
    def __init__(self, master):
        self.master = master
        master.title("Drone GUI")
        
        ## Initialising framework of GUI
        frame1 = Frame(master, height = 350, width = 210, bd = 2, relief = "sunken")
        frame1.grid(row = 10, column = 2, rowspan = 6)
        frame1.grid_propagate(False)
        
        ## Buttons
        self.select_target_button = Button(master, text="Select target", command=self.select_target)
        self.select_target_button.grid(row = 4, column = 2)

        self.start_auto_button = Button(master, text="Start autonomous flight", command=self.start_autonomous)
        self.start_auto_button.grid(row = 5, column = 2)
        self.approach_button = Button(master, text="Approach target", command=self.set_approach)
        self.approach_button.grid(row = 6, column = 2)

        self.e1 = Entry(master)
        self.e1.grid(row=7, column = 2)
        self.e1.bind("<Return>", self.get_target_distance)

        self.close_button = Button(master, text="Close", command=self.close_gui)
        self.close_button.grid(row = 8, column = 2)

        self.battery_label = Label(master, text = "Battery level: NA")
        self.battery_label.grid(row = 10, column = 2)

        self.x_distance_label = Label(master, text = "X Distance: NA")
        self.x_distance_label.grid(row = 11, column = 2)

        self.y_distance_label = Label(master, text = "Y Distance: NA")
        self.y_distance_label.grid(row = 12, column = 2)

        self.z_distance_label = Label(master, text = "Z Distance: NA")
        self.z_distance_label.grid(row = 13, column = 2)

        self.yaw_distance_label = Label(master, text = "Yaw angle: NA")
        self.yaw_distance_label.grid(row = 14, column = 2)

        self.flight_status_label = Label(master, text = "Autonomous mode: 0")
        self.flight_status_label.grid(row = 15, column = 2)

        header_label = Label(master, text="Choosing target for drone")
        header_label.grid(row = 1, column = 5)

        self.bb_data = (None, None, None, None)

        self.image_label = Label(text = "", height = 720, width = 1280)
        # self.image_label = Label(text = "", height = 480, width = 640)
        # self.image_label = Label(text = "", height = 448, width = 800)
        # self.image_label = Label(text = "", height = 1200, width = 1600)
        self.image_label.grid(row = 3, column = 5,  columnspan = 15, rowspan = 15)

        self.control_status = 0
        self.xTarget = 0

        ## Initialising variables for selecting target
        self.imgClick = False
        self.bridge = CvBridge()
        self.enable_video_stream = None
        self.prev_img = None
        self.select_target_bool = False
        
        self.circle_center = [None, None]

        self.pos_x = -1
        self.pos_y = -1

        # Subscribers
        self.battery_sub = rospy.Subscriber('/dji_sdk/battery_state', BatteryState, self.update_battery_label)
        self.target_sub = rospy.Subscriber("/target", Twist, self.draw_target)
        self.distance_error_sub = rospy.Subscriber("/distance_error", Point, self.update_distance_error)
        self.distance_sub = rospy.Subscriber('/dtu_controller/current_frame_pose', Twist, self.update_distance_label)
        self.image_sub = rospy.Subscriber('/camera/image_decompressed', SensorImage, self.image_subscriber_callback)

        # Publishers
        self.gui_target_pub = rospy.Publisher('/gui_target', Point , queue_size=1)
        self.frame_pub = rospy.Publisher('/frame_num', UInt8 , queue_size=1)
        self.control_status_msg = rospy.Publisher('/target_tracking_msg', UInt8, queue_size= 1)

        rospy.init_node('gui', anonymous=False)

        rospy.Timer(rospy.Duration(1.0/30.0), self.control_status_publish_callback)

        rospy.loginfo("GUI initialised")

    def save_pos(self, event): 
        ## updating the position of the target point from position of mouse click on image 
        self.pos_x = event.x
        self.pos_y = event.y

    def get_target_distance(self, entry):
        val = int(eval(self.e1.get()))
        if( val < 1 or val > 5):
            self.e1.delete(0,10)
            self.e1.insert("Invalid value")
        else:
            self.xTarget = val

    def close_gui(self):
        self.control_status = 0
        msg = UInt8()
        for i in range(1,10):
            msg.data = self.control_status
            self.control_status_msg.publish(msg)
            rospy.sleep(rospy.Duration(0.05))
        self.master.quit()

    def control_status_publish_callback(self, time_event):
        self.flight_status_label.configure( text = "Autonomous mode: {}".format(self.control_status))
        msg = UInt8()
        msg.data = self.control_status
        self.control_status_msg.publish(msg)

    def start_autonomous(self):
        if( self.control_status < 1 and self.circle_center[0] != None):
            self.control_status = 1
            rospy.loginfo("Autonomous tracking started")
        else:
            rospy.loginfo("No target selected")

    def set_approach(self):
        if( self.control_status > 0 and self.xTarget != 0):
            self.control_status = 1 + self.xTarget
        elif( self.xTarget == 0):
            rospy.loginfo("Approach distance not set")
        else:
            rospy.loginfo("No target selected or autonomous control not set")

    def image_subscriber_callback(self, image):
        cv_image = CvBridge().imgmsg_to_cv2(image, "rgb8")
        cv2.circle(cv_image, (640, 360), 1, (255,0,0), 10)
        if self.circle_center[0] != None:
            cv2.circle(cv_image, (int(self.circle_center[0]), int(self.circle_center[1])), 3, self.circ_color, 10)
            if self.bb_data[0] != None:
                cv2.rectangle(cv_image, (self.bb_data[0], self.bb_data[1]), (self.bb_data[0] + self.bb_data[2], self.bb_data[1] + self.bb_data[3]), self.circ_color, 2)
        self.img = Image.fromarray(cv_image)

    def draw_target(self,data):
        self.circle_center = [data.linear.x, data.linear.y]
        if data.linear.z:
            self.circ_color = (0,255,0)
        else:
            self.circ_color = (255,0,0)
        if data.angular.z:
            self.bb_data = (int(data.linear.x - data.angular.x/2), int(data.linear.y - data.angular.y/2), int(data.angular.x), int(data.angular.y))
        else:
            self.bb_data = (None, None, None, None)


    def update_image(self):
        ## Updating the image from the 'drone_cam_sub.py', if it's new. The update is automatic with a frequency 20 Hz (50 ms)
        frequency = 30.0
        try:
            if self.img != self.prev_img: 
                self.imgtk = ImageTk.PhotoImage(self.img)
                self.image_label.pic = self.imgtk
                self.image_label.configure(image=self.imgtk)
                self.prev_img = self.img
        except:
            print("Image not updated")
        self.enable_video_stream = self.image_label.after(int(1000/frequency), self.update_image)

    def select_target(self):
        ## Allows the user to select target, and interrupt selection if wanted
        if not self.select_target_bool:
            rospy.loginfo( "User is selecting target")
            self.select_target_bool = True
            self.imgClick = True
            self.select_target_button.configure(text = "Cancel")
            self.image_label.bind("<Button-1>", self.target_selected)
            self.image_label.configure(cursor="dotbox")
        else:
            rospy.loginfo("User cancelled selection")
            self.select_target_bool = False
            self.imgClick = False
            self.select_target_button.configure(text="Select target")
            self.image_label.unbind("<Button-1>")
            self.image_label.configure(cursor="")  

    def target_selected(self, event):
        ## Once target has been selected, variables and functions need to be reset. 
        ## By un-commenting line 158 control will be disabled, once autonomous flight is enabled 
        ## (For now it is possible to interfere with the drone by using the keyboard)
        self.select_target_bool = False
       
        rospy.loginfo("User selected target")
        self.imgClick = False
        self.save_pos(event)
        self.publish_pos()
        self.update_image()
        self.select_target_button.configure(text="Select target")
        self.image_label.unbind("<Button-1>")
        self.image_label.configure(cursor="") 
        
    def update_distance_label(self, data):
        self.x_distance_label.configure( text = 'X Distance:\n{:02.2f} m'.format(data.linear.x) )
        self.yaw_distance_label.configure( text = 'Yaw angle:\n{:02.2f} deg'.format(data.angular.z * 180.0/np.pi) )

    def update_distance_error(self, data):
        # self.x_distance_label.configure( text = 'X Distance:\n{:02.2f} m'.format(data.x))
        self.y_distance_label.configure( text = 'Y Distance:\n{:02.2f} m'.format(data.y))
        self.z_distance_label.configure( text = 'Z Distance:\n{:02.2f} m'.format(data.z))

    def update_battery_label(self, data):
        self.battery_label.configure( text = 'Battery level:\n{} %'.format(data.percentage))

    def publish_pos(self):
        #publishing the position of the target position in pixels
        if not rospy.is_shutdown():
            p = Point()
            p.x = self.pos_x
            p.y = self.pos_y
            p.z = 0
            self.gui_target_pub.publish(p)
            rospy.loginfo("New Gui target published (%d, %d)", self.pos_x, self.pos_y)



## sizing the gui window and initialising
root = Tk()
root.geometry('1700x850')

gui = DroneGUI(root)
gui.update_image()

col_count, row_count = root.grid_size()
for col in xrange(col_count):
    root.grid_columnconfigure(col, minsize=40)

for row in xrange(row_count):
    root.grid_rowconfigure(row, minsize=20)

root.mainloop()