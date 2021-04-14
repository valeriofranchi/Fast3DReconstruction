#! /usr/bin/env python

import numpy as np
from queue import Queue
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import rospy
import rosbag
from prophesee_event_msgs.msg import Event, EventArray
from reconstruction_3d_event_camera_pkg.msg import Line
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2
import sys
import math 
import colorsys
import struct 

class ProcessEvents:
    def __init__(self):
        # frame size 
        self.width = rospy.get_param("~width")
        self.height =  rospy.get_param("~height")

        # denoising parameters
        self.acc_time = rospy.get_param("~acc_time")
        self.neighbours = rospy.get_param("~neighbours")
        self.tail_length = rospy.get_param("~tail_length")
        self.min_density = rospy.get_param("~min_density")

        # raw events topic name
        self.events_topic_name = rospy.get_param("~events_topic_name")

        # calibration parameters 
        self.c_x = rospy.get_param("~cx")
        self.c_y = rospy.get_param("~cy")
        self.f_x = rospy.get_param("~fx")
        self.f_y = rospy.get_param("~fy")
        self.l_cy = rospy.get_param("~l_cy")
        self.l_cz = rospy.get_param("~l_cz")
        self.d = rospy.get_param("~dist_to_wall")

        # camera frame id 
        self.camera_frame_id = rospy.get_param("/camera_frame_broadcaster_node/camera_centre_frame_id")            
        
        # time between each frame i n microseconds
        self.frame_time = np.floor(1e6 / rospy.get_param("~frame_rate"))
        self.frame_num = 1

        # visualisation parameters
        self.dilate = rospy.get_param("~dilate_line")
        self.visualise = rospy.get_param("~visualise_images")
        self.pc_scale = rospy.get_param("~pointcloud_scale_factor")

        # thread-safe queue implementation 
        self.events = Queue(maxsize=0)
        self.X = Queue(maxsize=0)
        self.Y = Queue(maxsize=0)
        self.P = Queue(maxsize=0)
        self.T = Queue(maxsize=0)

        # data for reconstruction
        self.data = []     

        # fields for the point cloud message 
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1),
            ]
        
        # publishers 
        if self.visualise:
            self.raw_event_pub = rospy.Publisher("/events/image/raw", Image, queue_size=1)
            self.denoised_pub = rospy.Publisher("/events/image/denoised", Image, queue_size=1)    
        self.line_array_pub = rospy.Publisher("/events/extracted_line", Line, queue_size=1)
        self.pcl_publisher = rospy.Publisher("/events/points", PointCloud2, queue_size=1)

        # laser x rotation 
        self.last_good_theta = [0.0]

        # point cloud visualization reset in milliseconds
        self.reset = rospy.get_param("~pointcloud_reset")
        self.reset_counter = 1

        # if not live, process using ros bag file, else create subscriber for raw event data 
        if rospy.get_param("~offline"):
            self.bag = rosbag.Bag(rospy.get_param("~bag_file"))
            self.process_using_bag()
        else:
            self.event_array_subscriber = rospy.Subscriber(self.events_topic_name, EventArray, self.event_cb)

    """
    Subscriber for EventArray topic.
    msg: EventArray message
    """
    def event_cb(self, msg):
        self.create_event_packet(msg.events)

    """
    Processes the rosbag and creates event packets for each message.
    """
    def process_using_bag(self):
        # run ros bag and extract messages
        for _, msg, _ in self.bag.read_messages(topics=[self.events_topic_name]):
            self.create_event_packet(msg.events)           

        # close ros bag
        self.bag.close()

    """
    Creates event packet from EventArray message.
    events: EventArray message
    """
    def create_event_packet(self, events):
        # check time to create frame from events
        for event in events:            
            
            # create frame 
            if event.ts.to_sec() > (self.frame_num * self.frame_time):
                self.frame_num += 1

                # add events inside the queue 
                X = []
                Y = []
                P = []
                for e in self.events.queue:
                    X.append(e.x)
                    Y.append(e.y)
                    polarity = 1 if e.polarity == True else -1
                    P.append(polarity)

                # extract line and reconstruct the point cloud 
                self.extract_line(np.array(X), np.array(Y), np.array(P))  
                
            else:
                # add events to queue, and remove elements if more than interval time    
                if len(self.events.queue) > 1:           
                    while (self.events.queue[-1].ts - self.events.queue[0].ts).to_sec() > self.acc_time:
                        self.events.get()
                self.events.put(event)

            # reset point cloud view 
            if int(event.ts.to_sec()) > self.reset_counter * self.reset:  
                self.reset_counter = self.reset_counter + 1
                self.data = []          

    """
    Calculates the angle of the laser plane given a laser line and reference column
    v_in:       Vertical coordinate pixel excited in image plane
    returns angle of the laser plane
    Note - if line at reference column is NaN, returned value will be NaN (probably)
    The wall is assumed to be parallel to the camera plane
    """
    def extract_theta(self, v_in):
        # Calculate theta
        tan_phi = (self.c_y - v_in) / self.f_y
        dy = self.d * tan_phi
        theta = np.arctan2(dy - self.l_cy, self.d - self.l_cz)

        # Return theta
        return theta
 

if __name__ == '__main__':
    # create node 
    rospy.init_node("line_extraction_node")

    # process events
    process_events = ProcessEvents()

    rospy.spin()