#!/usr/bin/env python3

import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray, MultiArrayDimension


if __name__ == "__main__":
    try:
        rospy.init_node ("display_measures")
        floatPub = rospy.Publisher ("/walls",Float32MultiArray, queue_size=1)

        r = rospy.Rate (25)

        floatMsg = Float32MultiArray ()

        dim2 = MultiArrayDimension ()
        dim2.size = 2
        dim2.stride = 2

        dim1 = MultiArrayDimension ()
        dim1.size = 4
        dim1.stride = dim1.size * dim2.stride

        dim0 = MultiArrayDimension ()
        dim0.size = 2
        dim0.stride = dim0.size * dim1.stride

        floatMsg.layout.dim = [dim0, dim1, dim2]
        t = 0
        while not rospy.is_shutdown ():
            t += 0.008
            floatMsg.data = [-1*t, -1.1, 
                            1*t, -1.1,
                            1*t,  -0.9,
                            -1*t,  -0.9,
                            -1*t, 0.9,
                            1*t, 0.9,
                            1*t, 1.1,
                            -1*t, 1.1]
            floatPub.publish (floatMsg)
            r.sleep ()
    except rospy.ROSInterruptException:
        pass
