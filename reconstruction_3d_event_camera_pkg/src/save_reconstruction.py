#! /usr/bin/env python 
import rospy
import rosbag
from sensor_msgs.msg import PointCloud2, Image
from reconstruction_3d_event_camera_pkg.msg import Line 

def pointcloud_cb(msg):
    global topic_names
    global bag

    bag.write(msg, topic_names[0])

def raw_cb(msg):
    global topic_names
    global bag

    bag.write(msg, topic_names[1])

def denoised_cb(msg):
    global topic_names
    global bag

    bag.write(msg, topic_names[2])

def line_cb(msg):
    global topic_names
    global bag

    bag.write(msg, topic_names[3])

if __name__ == '__main__':
    global topic_names 

    # initialize node 
    rospy.init_node("save_reconstruction_node")

    topics = rospy.get_param("~topics")[0]
    topic_names = topics.keys()
    
    # set up subscribers 
    if topics[topic_names[0]] == 1:
        pc_sub = rospy.Subscriber(topic_names[0], PointCloud2, pointcloud_cb)
    
    if topics[topic_names[1]] == 1:
        raw_sub = rospy.Subscriber(topic_names[1], Image, raw_cb)

    if topics[topic_names[2]] == 1:
        denoised_sub = rospy.Subscriber(topic_names[2], Image, denoised_cb)

    if topics[topic_names[3]] == 1:
        line_sub = rospy.Subscriber(topic_names[3], Line, line_cb)
    
    # get bag file from ROS parameters and open bag file for writing 
    bag_file = rospy.get_param("~events_bag_file")
    bag = rosbag.Bag(bag_file, 'w')  

    rospy.spin()