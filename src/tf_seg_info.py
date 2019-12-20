#!/usr/bin/env python

import rospy
from dylan_msc.msg import obj
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

import numpy as np
from scipy import linalg

class Transformer():
    def __init__(self):
        rospy.init_node('tf_seg_info', anonymous=True)
        rospy.Subscriber("/obj_info_raw", obj, self.info_cb)
        rospy.Subscriber("/om_with_tb3/odom", Odometry, self.tf_cb)
        self.tf_info_pub = rospy.Publisher("/obj_info_tf", obj, queue_size=5)

        self.qBE = np.zeros((4, 1))
        self.rhoBE = 0
        self.nBE = np.zeros((3, 1))
        self.lambdaBE = 0
        self.LambdaBE = np.zeros((3, 3))
        self.TBE = np.zeros((3, 3))
        self.TEB = np.zeros((3, 3))
        self.sBE = np.zeros((3, 1))

        self.cent_pt = np.zeros((3, 1))
        self.min_pt = np.zeros((3, 1))
        self.max_pt = np.zeros((3, 1))
        self.tf_cent_pt = np.zeros((3, 1))
        self.tf_min_pt = np.zeros((3, 1))
        self.tf_max_pt = np.zeros((3, 1))

        self.TBC = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
            ])


    def info_cb(self, msg):
        self.cent_pt = np.array([
            [msg.centroid.x],
            [msg.centroid.y],
            [msg.centroid.z]
            ])

        self.min_pt = np.array([
            [msg.min.x],
            [msg.min.y],
            [msg.min.z]
            ])

        self.max_pt = np.array([
            [msg.max.x],
            [msg.max.y],
            [msg.max.z]
            ])

        self.tf_cent_pt = np.dot(self.TEB, np.dot(self.TBC, self.cent_pt)) + self.sBE
        self.tf_min_pt = np.dot(self.TEB, np.dot(self.TBC, self.min_pt)) + self.sBE
        self.tf_max_pt =  np.dot(self.TEB, np.dot(self.TBC, self.max_pt)) + self.sBE

        self.tf_obj = obj()
        self.tf_obj.index = msg.index
        self.tf_obj.centroid.x = self.tf_cent_pt[0]
        self.tf_obj.centroid.y = self.tf_cent_pt[1]
        self.tf_obj.centroid.z = self.tf_cent_pt[2]

        self.tf_obj.min.x = self.tf_min_pt[0]
        self.tf_obj.min.y = self.tf_min_pt[1]
        self.tf_obj.min.z = self.tf_min_pt[2]

        self.tf_obj.max.x = self.tf_max_pt[0]
        self.tf_obj.max.y = self.tf_max_pt[1]
        self.tf_obj.max.z = self.tf_max_pt[2]

        self.tf_info_pub.publish(self.tf_obj)

       


    def tf_cb(self, msg):
        self.qBE = np.array([
            [msg.pose.pose.orientation.w],
            [msg.pose.pose.orientation.x],
            [msg.pose.pose.orientation.y],
            [msg.pose.pose.orientation.z]
            ])
        self.rhoBE = 2*np.arccos(self.qBE[0])
        self.nBE = self.qBE[1:4]/np.sin(0.5*self.rhoBE)
        self.lambdaBE = self.rhoBE*self.nBE
        print(self.lambdaBE, "\n\n")
        self.LambdaBE = np.array([
            [0, -self.lambdaBE[2], self.lambdaBE[1]],
            [self.lambdaBE[2], 0, -self.lambdaBE[0]],
            [-self.lambdaBE[1], self.lambdaBE[0], 0]
            ])
        self.TBE = linalg.expm(-self.LambdaBE)
        self.TEB = self.TBE.transpose()
        print(self.TEB, "\n\n")

        self.sBE = np.array([
            [msg.pose.pose.position.x],
            [msg.pose.pose.position.y],
            [msg.pose.pose.position.z]
            ])


if __name__ == '__main__':
    print("Loading transformation node...")
    transformer = Transformer()
    print("Transformation node loaded.")
    rospy.spin() 
