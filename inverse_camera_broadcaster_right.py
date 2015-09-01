#!/usr/bin/env python

import roslib
import rospy
import tf
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped, PoseStamped
import tfx
import pickle
import sys

CAP_OFFSET = -0.009#-0.011#-0.00264
SKETCH_OFFSET = 0#-0.011

class InverseCamBroadcaster:
    def __init__(self):
        print("init")
        self.have_transform = False
        self.camera_transform = None
        rospy.init_node("camera_inverse_broadcaster", anonymous=True)
        self.pub = rospy.Publisher("/tf", tfMessage)
        

    def get_transform_callback(self, data):
        if not self.have_transform:
            self.have_transform = True
            f = open("/home/davinci2/catkin_ws/src/davinci_vision/launch/BC_registration/camera_frame.p", "wb")
            pickle.dump(data, f)
            f.close()
        
    def listener(self):
        rospy.Subscriber("/BC/chessboard_pose", PoseStamped, self.get_transform_callback)
        while not self.have_transform and not rospy.is_shutdown():
            rospy.sleep(0.1)
            print("waiting for transform")
        self.broadcast()

    def broadcast(self):
        f = open("/home/davinci2/catkin_ws/src/davinci_vision/launch/BC_registration/camera_frame.p", "rb")
        self.camera_transform = pickle.load(f)
        f.close()
        #SOMETIMES NEED TO INVERT X & Y AXES; just change from 1 to -1 and vice versa
        offset = tfx.transform([SKETCH_OFFSET, 0, CAP_OFFSET], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.camera_transform = tfx.transform(self.camera_transform).as_transform() * offset
        transform = tfx.inverse_tf(self.camera_transform)
        pt = transform.translation
        rot = transform.rotation
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.transform.rotation.x = rot.x
        msg.transform.rotation.y = rot.y
        msg.transform.rotation.z = rot.z
        msg.transform.rotation.w = rot.w
        msg.child_frame_id = "left_BC"   
        msg.transform.translation.x = pt.x
        msg.transform.translation.y = pt.y
        msg.transform.translation.z = pt.z
        msg.header.frame_id = "registration_brick_right"
        msg.header.stamp = rospy.Time.now()
        print('boo')
        while not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            self.pub.publish([msg])
            rospy.sleep(0.1)
        
if __name__ == '__main__':
    c = InverseCamBroadcaster()
    if sys.argv[1] == "file":
        print("file")
        c.broadcast()
    else:
        print("listener")
        c.listener()
    