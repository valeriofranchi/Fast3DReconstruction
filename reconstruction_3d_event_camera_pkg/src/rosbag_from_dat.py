#! /usr/bin/env python
import rospy
from reconstruction_3d_event_camera_pkg.msg import Event, EventArray
import dat_conversion
from std_msgs.msg import Header
import rosbag
import struct 
import copy

def next_byte(b, file, chunk):
    if len(b) == 0:
        b = file.read(chunk)
    byte = None if len(b) == 0 else b[0]
    b = b[1:]
    return [byte, b]

def read_file(path, buffer_size):
    global bag 

    chunk = 1024

    count = 0
    events = []

    f = open(path, 'rb')

    try:

        b = b''

        [byte, b] = next_byte(b, f, chunk)

        hit_new_line = True
        end_of_header = False
        do_skip_one = False

        current_comment = ""

        current_data = []
        event_buffer = []

        while byte != None:            

            if do_skip_one:
                do_skip_one = False
                [byte, b] = next_byte(b, f, chunk)
                continue

            # Do something
            if not end_of_header:
                # Process header
                if hit_new_line:
                    hit_new_line = False

                    if byte == "%":
                        pass 
                    else:
                        end_of_header = True
                        do_skip_one = True
                if not end_of_header:
                    if byte == "\n":
                        current_comment = ""
                        hit_new_line = True
                    else:
                        current_comment += byte
                        hit_new_line = False
            else:
                # Process data
                current_data.append(byte)
                if len(current_data) == 8:

                    # Extract data from 8 bytes
                    ts, dat = struct.unpack("<LL", bytearray(current_data))
                    x = (dat & 0x00003FFF)
                    y = (dat & 0x0FFFC000) >> 14
                    p = 1 if ((dat & 0x10000000) >> 28) else -1

                    if count < buffer_size:
                        if p < 1:
                            p = 0
                        event_buffer.append([x,y,ts,p])
                        count = count + 1
                    else:
                        add_to_bag(event_buffer)
                        event_buffer = []
                        count = 0
                    
                    current_data = []
            
            [byte, b] = next_byte(b, f, chunk)
        
    finally:
        if len(event_buffer) > 0:
            add_to_bag(event_buffer)
       
        bag.close()
        f.close()
        
    return events

"""
Add events to bag from .dat file 
"""
def add_to_bag(events):
    global height 
    global width   
    global i 
    global topic
    global bag 

   # create event array 
    event_msg = EventArray(
        events=[Event(x=x,y=y,ts=rospy.Time.from_sec(ts * pow(10,-3)),polarity=p) for x,y,ts,p in events],
        header=Header(seq=i, stamp=rospy.Time.now()),
        height=height, 
        width=width
    )

    # increment sequence number 
    i = i + 1

    # write to bag 
    bag.write(topic, event_msg)  

if __name__ == '__main__':
    global height
    global width
    global i 
    global topic
    global bag

    # initialize node 
    rospy.init_node("generate_rosbag_node")

    # initialize sequence number 
    i = 0

    # load parameters from ROS Param server 
    topic =  rospy.get_param("/reconstruction_node/events_topic_name")
    buffer_size =  rospy.get_param("/reconstruction_node/buffer_size")
    height = rospy.get_param("/reconstruction_node/height")
    width = rospy.get_param("/reconstruction_node/width")

    dat_file =  rospy.get_param("~events_dat_file")
    bag_file = rospy.get_param("~events_bag_file")

    # open bag file for writing 
    bag = rosbag.Bag(bag_file, 'w')

    # write into bag file
    read_file(dat_file, buffer_size)

    rospy.spin()