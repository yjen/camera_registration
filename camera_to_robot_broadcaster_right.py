#!/usr/bin/env python

import roslib
import rospy
import tf
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped
import tfx

class CaroBroadcaster:
    def __init__(self):
        print("init")

    def publishLatchTransforms(self):
        rospy.init_node('caro_broadcaster', anonymous=True)
        self.publishLatchTransform("right")
        while not rospy.is_shutdown():
            rospy.spin()

    def publishLatchTransform(self, arm):
        if arm == 'left':
            self.pub_tf_left = rospy.Publisher("/tf", tfMessage, queue_size=1, latch=True)
        else:
            self.pub_tf_right = rospy.Publisher("/tf", tfMessage, queue_size=1, latch=True)
            
        f = open("/home/davinci2/catkin_ws/src/davinci_vision/launch/BC_registration/transform_" + arm + \
            ".txt", "r")
        transform = f.readline().split()
        f.close()
        print(transform)
        # Send static link transforms
        msg = TransformStamped()

        msg.header.stamp = rospy.Time.now()
        msg.transform.rotation.x =  float(transform[3])
        msg.transform.rotation.y =  float(transform[4])
        msg.transform.rotation.z =  float(transform[5])
        msg.transform.rotation.w =  float(transform[6])
        msg.child_frame_id = "left_BC"   
        msg.transform.translation.x = float(transform[0])
        msg.transform.translation.y = float(transform[1])
        msg.transform.translation.z = float(transform[2])
        if arm == 'left':
            # msg.header.frame_id = "two_psm_base_link"
            msg.header.frame_id = "two_remote_center_link"
        else:
            msg.header.frame_id = "one_remote_center_link"
        while not rospy.is_shutdown():
                msg.header.stamp = rospy.Time.now()
                self.pub_tf_right.publish([msg])
                rospy.sleep(0.5)
        
if __name__ == '__main__':
    c = CaroBroadcaster()
    c.publishLatchTransforms()