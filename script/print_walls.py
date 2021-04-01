#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import matplotlib.pyplot as plt

class PrintWalls:
    def __init__ (self):
        rospy.init_node ("print_walls")

        self.fig = plt.figure ()
        self.wall_sub = rospy.Subscriber ("/walls", Float32MultiArray, self.wallsCallback)

        plt.show ()

    def wallsCallback (self, wallMsg):
        walls = np.array (wallMsg.data).reshape ((2,-1,2))

        self.draw (walls[0,:,:],walls[1,:,:])

    def draw (self, wall1, wall2):
        self.fig.clear ()
        
        plt.plot (wall1[:,0], wall1[:,1])
        plt.plot (wall2[:,0], wall2[:,1])
        plt.draw ()

    def spin(self):
        rospy.spin ()



if __name__ == "__main__":
    try:
        pw = PrintWalls()
        pw.spin ()

    except rospy.ROSInterruptException:
        pass