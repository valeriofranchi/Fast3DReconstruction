#! /usr/bin/env python 
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from reconstruction_3d_event_camera_pkg.msg import Line
import tf 

def broadcast_laser(msg):
    global broadcaster 
    global i 
    global offset 
    global camera_frame
    global laser_frame
    
    # initialize transform with parent and child frames 
    laser_tf_msg = TransformStamped()
    laser_tf_msg.header.stamp = rospy.Time.now()
    laser_tf_msg.header.frame_id = camera_frame
    laser_tf_msg.child_frame_id = laser_frame
    laser_tf_msg.header.seq = i

    # positional offset
    laser_tf_msg.transform.translation.x = float(offset[0])
    laser_tf_msg.transform.translation.y = float(offset[1])
    laser_tf_msg.transform.translation.z = float(offset[2])

    # rotational offset 
    quat_rot = tf.transformations.quaternion_from_euler(
        float(msg.theta), float(0.0), float(0.0))
    laser_tf_msg.transform.rotation.x = quat_rot[0]
    laser_tf_msg.transform.rotation.y = quat_rot[1]
    laser_tf_msg.transform.rotation.z = quat_rot[2]
    laser_tf_msg.transform.rotation.w = quat_rot[3]

    # publish transform and increase sequence number 
    broadcaster.sendTransform(laser_tf_msg)
    i = i + 1

if __name__ == '__main__':
    global broadcaster
    global offset 
    global camera_frame
    global laser_frame
    global i 
    
    # initialize node
    rospy.init_node("my_dynamic_transform_broadcaster")

    # create transform broadcaster 
    broadcaster = tf2_ros.TransformBroadcaster()
    
    # initialize sequence number 
    i = 0

    # positional offset between camera and laser link  
    offset = [0.0, rospy.get_param("/reconstruction_node/l_cy"), 
        rospy.get_param("/reconstruction_node/l_cz")]

    # camera and laser frames 
    camera_frame = rospy.get_param("/camera_frame_broadcaster_node/camera_mount_frame_id")
    laser_frame = rospy.get_param("/camera_frame_broadcaster_node/laser_frame_id")

    # name of line topic 
    sub = rospy.Subscriber(rospy.get_param("/reconstruction_node/theta_topic"), Line, broadcast_laser)
    rospy.spin()