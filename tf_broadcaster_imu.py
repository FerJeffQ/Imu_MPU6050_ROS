#!/usr/bin/env python3


import rospy
import math
import geometry_msgs.msg
from tf.broadcaster import TransformBroadcaster
from sensor_msgs.msg import Temperature, Imu

from geometry_msgs.msg import TransformStamped

import tf
from tf.transformations import euler_from_quaternion

def handle_imu_pose(msg):
    br = tf.TransformBroadcaster()

    
    orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    q0 = msg.orientation.x
    q1 = msg.orientation.y
    q2 = msg.orientation.z
    q3 = msg.orientation.w
    

    odom_quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    br.sendTransform((0, 0, 0), odom_quat,rospy.Time.now(),"imu_link", "tf_plano")
    
    

if __name__ == '__main__':
      print("Se inicio el nodo de publicacion TF_imu....")
      rospy.init_node('tf_broadcaster_imu')
      rospy.Subscriber('/imu/data', Imu, handle_imu_pose)
      rospy.spin()
