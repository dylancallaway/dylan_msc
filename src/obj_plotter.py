#!/usr/bin/env python

import rospy
from dylan_msc.msg import obj
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
from nav_msgs.msg import Odometry


class Plotter():
    def __init__(self):
        rospy.init_node('obj_plotter', anonymous=True)
        rospy.Subscriber("/obj_info_tf", obj, self.sub_cb)
        rospy.Subscriber("/om_with_tb3/odom", Odometry, self.bot_cb)

        self.bot_x = 0
        self.bot_y = 0
        self.bot_z = 0

        plt.ion()
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)
        self.graph = self.ax.scatter([], [], [], c='r', marker='o')
        axes_size = 2
        self.ax.set_xlim(0, axes_size)
        self.ax.set_ylim(-axes_size, axes_size)
        self.ax.set_zlim(-0.15, axes_size)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.fig.canvas.draw()

        self.cent_x = []
        self.cent_y = []
        self.cent_z = []
        self.min_x = []
        self.min_y = []
        self.min_z = []
        self.max_x = []
        self.max_y = []
        self.max_z = []
        self.verts = []

        self.final_cent_x = []
        self.final_cent_y = []
        self.final_cent_z = []
        self.final_min_x = []
        self.final_min_y = []
        self.final_min_z = []
        self.final_max_x = []
        self.final_max_y = []
        self.final_max_z = []
        self.final_verts = []

        self.bb = []

        self.road_0 = [0, 2, -0.15]
        self.road_1 = [0, -2, -0.15]
        self.road_2 = [2, 2, -0.15]
        self.road_3 = [2, -2, -0.15]
        self.road_points = (
            [[self.road_1, self.road_3, self.road_2, self.road_0]])
        self.road_plot = Poly3DCollection(
            self.road_points, facecolors='green', linewidths=1, edgecolors='r', alpha=0.5)
        self.ax.add_collection3d(self.road_plot)

    def bot_cb(self, msg):
        self.bot_x = msg.pose.pose.position.x
        self.bot_y = msg.pose.pose.position.y
        self.bot_z = msg.pose.pose.position.z


    def sub_cb(self, msg):
        # rospy.loginfo("recv: %d", msg.index)

        # if (msg.min.y > -0.75 and msg.min.y < 0.75) or (msg.max.y > -0.75 and msg.max.y < 0.75):
        if True:
            if msg.index == 0:
                # Save entire arrays
                self.final_cent_x = self.cent_x
                self.final_cent_y = self.cent_y
                self.final_cent_z = self.cent_z
                self.final_min_x = self.min_x
                self.final_min_y = self.min_y
                self.final_min_z = self.min_z
                self.final_max_x = self.max_x
                self.final_max_y = self.max_y
                self.final_max_z = self.max_z
                self.final_verts = self.verts

                # Reset arrays
                self.cent_x = []
                self.cent_y = []
                self.cent_z = []
                self.min_x = []
                self.min_y = []
                self.min_z = []
                self.max_x = []
                self.max_y = []
                self.max_z = []
                self.verts = []

            # If not a new message append the info
            self.cent_x.append(msg.centroid.x)
            self.cent_y.append(msg.centroid.y)
            self.cent_z.append(msg.centroid.z)
            self.min_x.append(msg.min.x)
            self.min_y.append(msg.min.y)
            self.min_z.append(msg.min.z)
            self.max_x.append(msg.max.x)
            self.max_y.append(msg.max.y)
            self.max_z.append(msg.max.z)
            p1 = [msg.min.x, msg.min.y, msg.min.z]
            p2 = [msg.max.x, msg.min.y, msg.min.z]
            p3 = [msg.max.x, msg.max.y, msg.min.z]
            p4 = [msg.min.x, msg.max.y, msg.min.z]
            p5 = [msg.min.x, msg.min.y, msg.max.z]
            p6 = [msg.max.x, msg.min.y, msg.max.z]
            p7 = [msg.max.x, msg.max.y, msg.max.z]
            p8 = [msg.min.x, msg.max.y, msg.max.z]

            self.verts.append([[p1, p2, p3, p4],
                               [p5, p6, p7, p8],
                               [p1, p2, p6, p5],
                               [p3, p4, p8, p7],
                               [p2, p3, p7, p6],
                               [p5, p8, p4, p1]])

    def animation_cb(self, unused_iterator):
        cent_x = self.final_cent_x
        cent_y = self.final_cent_y
        cent_z = self.final_cent_z
        verts = self.final_verts

        # for i in range(0, len(self.bb)):
            # self.bb[i].remove()

        self.bb = []

        cent_x.append(self.bot_x)
        cent_y.append(self.bot_y)
        cent_z.append(self.bot_z)

        self.graph._offsets3d = (cent_x, cent_y, cent_z)

        for i in range(0, len(verts)):
            self.bb.append(Poly3DCollection(
                verts[i], facecolors='red', linewidths=2, edgecolors='r', alpha=0.25))
            self.ax.add_collection3d(self.bb[i])

        return self.graph


if __name__ == '__main__':
    plotter1 = Plotter()
    ani = animation.FuncAnimation(
        plotter1.fig, plotter1.animation_cb, interval=200, blit=False)
    plt.show(block=True)
    # rospy.spin() not need because of plt.show()
