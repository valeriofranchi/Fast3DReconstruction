#! /usr/bin/env python

import rospy
import tf 
import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('camera_static_broadcaster')

    # positional and rotation offsets between the transform frames 
    mount_pos = rospy.get_param("~base_to_cam_mount_pos_offset")
    mount_rot = rospy.get_param("~base_to_cam_mount_rot_offset")
    centre_pos = rospy.get_param("~cam_mount_to_centre_pos_offset")
    centre_rot = rospy.get_param("~cam_mount_to_centre_rot_offset")

    # base and camera frames
    base_id = rospy.get_param("~base_frame_id")
    mount_id = rospy.get_param("~camera_mount_frame_id")
    centre_id = rospy.get_param("~camera_centre_frame_id")

    # initialize static transform broadcaster and transforms 
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    base_cam_tf_msg = TransformStamped()
    cam_centre_tf_msg = TransformStamped()

    # initialize sequence numbers 
    i = 0
    j = 0

    # frequency of loop 
    rate = rospy.Rate(10.0)

    # transform from robot base to camera mount point     
    base_cam_tf_msg.header.stamp = rospy.Time.now()
    base_cam_tf_msg.header.frame_id = base_id
    base_cam_tf_msg.child_frame_id = mount_id
    base_cam_tf_msg.header.seq = i

    # positional offset 
    base_cam_tf_msg.transform.translation.x = float(mount_pos[0])
    base_cam_tf_msg.transform.translation.y = float(mount_pos[1])
    base_cam_tf_msg.transform.translation.z = float(mount_pos[2])

    # rotational offset 
    quat_rot_mount = tf.transformations.quaternion_from_euler(
        float(mount_rot[0]), float(mount_rot[1]), float(mount_rot[2]))
    base_cam_tf_msg.transform.rotation.x = quat_rot_mount[0]
    base_cam_tf_msg.transform.rotation.y = quat_rot_mount[1]
    base_cam_tf_msg.transform.rotation.z = quat_rot_mount[2]
    base_cam_tf_msg.transform.rotation.w = quat_rot_mount[3]

    # transform from camera mount point to camera centre 
    cam_centre_tf_msg.header.stamp = rospy.Time.now()
    cam_centre_tf_msg.header.frame_id = mount_id
    cam_centre_tf_msg.child_frame_id = centre_id
    cam_centre_tf_msg.header.seq = j

    # positional offset
    cam_centre_tf_msg.transform.translation.x = float(centre_pos[0])
    cam_centre_tf_msg.transform.translation.y = float(centre_pos[1])
    cam_centre_tf_msg.transform.translation.z = float(centre_pos[2])

    # rotational offset 
    quat_rot_centre = tf.transformations.quaternion_from_euler(
        float(centre_rot[0]), float(centre_rot[1]), float(centre_rot[2]))
    cam_centre_tf_msg.transform.rotation.x = quat_rot_centre[0]
    cam_centre_tf_msg.transform.rotation.y = quat_rot_centre[1]
    cam_centre_tf_msg.transform.rotation.z = quat_rot_centre[2]
    cam_centre_tf_msg.transform.rotation.w = quat_rot_centre[3]

    while not rospy.is_shutdown():
        # broadcast both 
        #broadcaster.sendTransform((float(mount_pos[0]), float(mount_pos[1]), float(mount_pos[2])), 
        #    (quat_rot_mount[0], quat_rot_mount[1], quat_rot_mount[2], quat_rot_mount[3]), 
        #    rospy.Time.now(), mount_id, base_id)

        #broadcaster.sendTransform((float(centre_pos[0]), float(centre_pos[1]), float(centre_pos[2])), 
        #    (quat_rot_centre[0], quat_rot_centre[1], quat_rot_centre[2], quat_rot_centre[3]), 
        #    rospy.Time.now(), centre_id, mount_id)

        # broadcast both transforms 
        broadcaster.sendTransform(base_cam_tf_msg)
        broadcaster.sendTransform(cam_centre_tf_msg)

        # add sequence numbers 
        i = i + 1
        j = j + 1

        # sleep 
        rate.sleep()

    rospy.spin()

