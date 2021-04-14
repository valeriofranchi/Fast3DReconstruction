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
    Converts numpy array of 3D points to PointCloud2 message.
    points: ndarray of x,y,z points 
    """ 
    def nparray_to_pointcloud2(self, points):
        # create colour bar and assign rgba values based on z value 
        min_z = np.nanmin(points[:,2]) * self.pc_scale
        max_z = np.nanmax(points[:,2]) * self.pc_scale
        diff = max_z - min_z
        
        # iterate over all points
        updated_points = []
        for point in points:
            # if point is not nan, add rgba value based on z coordinate 
            point = np.float32(point * self.pc_scale)
            if not np.isnan(point).any() and point[2] > 0:
                h = (point[2] - min_z) / float(diff) # min max scaling 
                r, g, b = colorsys.hsv_to_rgb(h, 0.5, 0.5) 
                rgba = struct.unpack('I', struct.pack('BBBB', int(b*255.0), int(g*255.0), int(r*255.0), 255))[0]
                updated_points.append([point[0], point[1], point[2], rgba])
        points = np.asarray(updated_points)

        header = Header(stamp=rospy.Time.now(), frame_id=self.camera_frame_id)
        pc_msg = point_cloud2.create_cloud(header, self.fields, points)
        return pc_msg

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

    """
    Reconstructs a line of excited pixels given the laser angle.
    u: numpy array of excited pixel columns (no NaN)
    v:  numpy array of excited pixel heights (may be NaN)
    theta: numpy array of angles of the laser plane
    returns numpy array (Nx3) of points in 3D space
    Note - no filtering for impossible depths is performed
    """
    def reconstruct_line(self, u, v, theta):
        # Calculate tan of theta and phi
        tan_theta = np.squeeze(np.tan(theta.astype(np.float64)))
        tan_phi = (self.c_y - v) / self.f_y

        # Calculate z offset
        z = (self.l_cy - self.l_cz * tan_theta) / (tan_phi - tan_theta)

        # Calculate x and y offset
        x = np.multiply(z, (self.c_x - u) / self.f_x)
        y = np.multiply(z, (self.c_y - v) / self.f_y)

        # Return [x, y, z] columns
        points = np.vstack((np.reshape(x, (1, -1)), np.reshape(y, (1, -1)), 
            np.reshape(z, (1, -1)))).T

        # convert numpy array to PointCloud2 message 
        msg = self.nparray_to_pointcloud2(points)

        # publish message 
        self.pcl_publisher.publish(msg)

    """
    Extracts line from event data (x,y,p).
    x: numpy array of x coordinates of events
    y: numpy array of y coordinates of events
    p: numpy array of polarity values of events
    """
    def extract_line(self, x, y, p):
        # get positive events
        x_pos = x[p==1]
        y_pos = y[p==1]

        if self.visualise:
            # create frame from raw data 
            frame = np.zeros((self.height, self.width))
            for j in range(len(x)):
                frame[y[j], x[j]] = 1
            
            # publish raw events image 
            self.publish_image(frame, "mono8")

        # get total occurrences in pixels
        bins_x = np.linspace(0,self.width-1, num=self.width, dtype=int) 
        bins_y = np.linspace(0,self.height-1, num=self.height, dtype=int) 
        H, _, _ = np.histogram2d(y, x, bins=(bins_y, bins_x))

        # get unique x values 
        x_unique = np.unique(x_pos)

        # initialize line
        line = np.empty((self.width, 1))
        line.fill(np.nan)

        # iterate over unique x values 
        for j in range(len(x_unique)):
            # get neighbourhood around x value 
            neighbourhood = y_pos[np.logical_and(x_pos > (x_unique[j] - self.neighbours), 
                x_pos < (x_unique[j] + self.neighbours))]

            # median of neighbourhood 
            y_val = np.median(neighbourhood)

            # get current x unique value 
            x_val = x_unique[j]

            # check if it has a tail
            if np.sum(p[x == x_val] == -1) > self.tail_length:
                line[x_val] = y_val

            # check density
            y_min = int(y_val) - self.neighbours
            x_min = x_val - self.neighbours
            y_max = int(y_val) + self.neighbours
            x_max = x_val + self.neighbours

            if y_min < 0:
                y_min = 0
            if y_max >= self.height:
                y_max = self.height - 1
            if x_min < 0:
                x_min = 0
            if x_max >= self.width:
                x_max = self.width - 1
            
            if np.sum(H[y_min: y_max, x_min: x_max]) > self.min_density:
                line[x_val] = y_val

        # create Line message and add non NaN points 
        line_msg = Line()
        line_msg.points = [int(-1) if np.isnan(p) else int(p[0]) for p in line]
        
        # extract theta
        theta = self.extract_theta(line[280])
        if not np.isnan(theta):
            self.last_good_theta = theta

        # publish line 
        line_msg.theta = self.last_good_theta
        self.line_array_pub.publish(line_msg)
        
        # append to the numpy array only non Nan values and the theta value 
        for i, l in enumerate(line):
            if not np.isnan(l):
                self.data.append(np.array([i,l[0],self.last_good_theta[0]]))
        
        # reconstruct point cloud
        if len(self.data) > 0:
            data = np.asarray(self.data)
            u = data[:,0]
            v = data[:,1]
            t = data[:,2]
            self.reconstruct_line(u, v, t)

        if self.visualise:
            # initialize white image 
            denoised = np.ones((self.height, self.width), dtype=np.uint8) 
            white = denoised.copy()

            # add red pixels to every non Nan value
            for i, k in enumerate(line):
                if math.isnan(k) == False:
                    denoised[int(k),i] = 0
            
            # dilate denoised image to visualise the line better 
            kernel = np.ones((3,3),np.uint8)
            eroded = cv2.erode(denoised, kernel, iterations=1)
            
            # dilated or not dilated line 
            bg_chans = eroded if self.dilate else denoised
            final = cv2.merge([bg_chans, bg_chans, white])

            # publish denoised events image 
            self.publish_image(final, "bgr8")
    
    '''
    Convert numpy array to sensor_msgs::Image and publish it
    frame: numpy array of image 
    image_encoding: Image encoding such mono8, bgr8, etc...  
    '''
    def publish_image(self, frame, image_encoding):
        npimage = np.uint8(frame*255)

        # MIT License Copyright (c) 2016 Eric Wieser         
        im = Image(encoding=image_encoding)

        if image_encoding == "mono8":
            im.height, im.width = npimage.shape
        elif image_encoding == "bgr8":
            im.height, im.width, _ = npimage.shape

        contig = np.ascontiguousarray(npimage)
        im.data = contig.tostring()
        im.step = contig.strides[0]
        im.is_bigendian = (
            frame.dtype.byteorder == '>' or 
            frame.dtype.byteorder == '=' and sys.byteorder == 'big'
            )
        
        if image_encoding == "mono8":
            self.raw_event_pub.publish(im)
        elif image_encoding == "bgr8":
            self.denoised_pub.publish(im)

if __name__ == '__main__':
    # create node 
    rospy.init_node("line_extraction_node")

    # process events
    process_events = ProcessEvents()

    rospy.spin()