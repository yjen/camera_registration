#!/usr/bin/python
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import rospy
from numpy.linalg import svd
from scipy.optimize import leastsq

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial, time
import pickle
import sys

import tfx



########

#CHANGE CONSTANTS TO CLASS VARS

#PLOTTING ONLY WORKS WITH MATPLOTLIB 1.1.1
#ros hydro
#numpy 1.9.2
#ubuntu 12.04

########


class RoboRegistration:
    
    def __init__(self, arm):
        print(arm)
        self.arm = arm
        self.pose_data = []
        self.recording_pose = False
        self.done_recording = False
        self.have_camera_transform = False
        self.camera_transform = None


    def record_pose_callback(self, data):
        if self.recording_pose:
            print(data)
            self.pose_data.append(data)
            #print(str(time.clock()) + 'pose')
            self.recording_pose = False

    def get_transform_callback(self, data):
        if not self.have_camera_transform:
            self.camera_transform = data
            self.have_camera_transform = True
            print("Got camera transform!")

    def plot(self, point, normal, min_x, max_x, min_y, max_y):
        d = -point.dot(normal)
        # xx, yy = np.meshgrid(range(-1, 2), range(-1, 2))
        xx, yy = np.meshgrid([min_x, max_x], [min_y, max_y])
        z = (-normal[0] * xx - normal[1] * yy -d) * 1. /normal[2]
        plt3d = plt.figure().gca(projection='3d')
        plt3d.set_xlim3d([-0.2, 0.2])
        plt3d.set_ylim3d([-0.2, 0.2])
        plt3d.set_zlim3d([-0.2, -0.1])
        plt3d.set_autoscale_on(False)
        plt3d.plot_surface(xx, yy, z, alpha=0.2, color='blue')
        x = []
        y = []
        z = []
        for pose in self.pose_data:
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)
            z.append(pose.pose.position.z)
        x = np.array(x)
        y = np.array(y)
        z = np.array(z)
        data = np.concatenate((x[:, np.newaxis], y[:, np.newaxis], z[:, np.newaxis]), axis=1)
        plt3d.scatter3D(*data.T, c='red')
        # for pose in self.pose_data:
        #     plt3d.plot(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        plt.show()

    def plane_fit(self, points):
        """
        p, n = planeFit(points)

        Given an array, points, of shape (d,...)
        representing points in d-dimensional space,
        fit an d-dimensional plane to the points.
        Return a point, p, on the plane (the point-cloud centroid),
        and the normal, n.
        """
        print(points)
        points = np.reshape(points, (np.shape(points)[0], -1)) # Collapse trialing dimensions
        print(points)
        print(points.shape[0])
        print(points.shape[1])
        assert points.shape[0] <= points.shape[1], "There are only {} points in {} dimensions.".format(points.shape[1], points.shape[0])
        ctr = points.mean(axis=1)
        x = points - ctr[:,np.newaxis]
        M = np.dot(x, x.T) # Could also use np.cov(x) here.
        return ctr, svd(M)[0][:,-1]

    def listener(self):
        rospy.init_node('robo_registration', anonymous=True)
        # dvrk_psm1/joint_position_cartesian is right arm
        pose_topic = ""
        if self.arm == "right":
            pose_topic = "dvrk_psm1/joint_position_cartesian"
        else:
            pose_topic = "dvrk_psm2/joint_position_cartesian"
        rospy.Subscriber(pose_topic, PoseStamped, self.record_pose_callback)   
        rospy.Subscriber("/BC/chessboard_pose", PoseStamped, self.get_transform_callback)

        

        save_camera_transform = raw_input("Do you want to save a new camera transform? (yes/no) ")

        if save_camera_transform == "yes":
            while not self.have_camera_transform and not rospy.is_shutdown():
                print("Waiting for camera transform")
                rospy.sleep(0.5)
            print("Saving camera transform in file")
            f = open("/home/davinci2/catkin_ws/src/davinci_vision/launch/BC_registration/camera_frame.p", "wb")
            pickle.dump(self.camera_transform, f)
            f.close()
        else:
            f = open("/home/davinci2/catkin_ws/src/davinci_vision/launch/BC_registration/camera_frame.p", "rb")
            self.camera_transform = pickle.load(f)
            f.close()

        raw_input("Remove chessboard; press any key when done")
        cols = int(raw_input("Enter number of columns on registration block: "))
        rows = int(raw_input("Enter number of rows on registration block: "))
        dimensions = [cols, rows]
        #will have to do both grippers
        print("Please start at top left corner and go row by row, moving from left to right. Use " + self.arm + \
            " gripper")
        print("First coordinate is x, second is y; (0, 0) is top left corner")
        # cols_even = cols % 2 == 0
        # rows_even = rows % 2 == 0
        
        # wrist_poses = []

        for j in range(rows):
            for i in range(cols):
                command = raw_input("Press r to record current pose (cell " + str(i) + ", " + str(j) + "): ")
                if command == "r":
                    self.recording_pose = True
                    print("Pose recorded!")
                    # wrist_poses.append(tfx.lookupTransform("two_tool_wrist_sca_shaft_link", "two_remote_center_link").msg.PoseStamped())
                

                # b = tfx.pose(transform)
                # b.name = None
                # b.as_tf() * self.pose_data[0]
                
        #LOLMAGICNUMBER        
        offset = tfx.transform([0, 0, -0.00483], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        trans_data = [tfx.pose(tfx.pose(pose).as_tf() * offset).msg.PoseStamped() for pose in self.pose_data]
        self.pose_data = trans_data
        # while not self.done_recording:
        #     command = raw_input("Press r to record current pose or q to finish: ")
        #     if command == "r":
        #         self.recording_pose = True
        #         print("Pose recorded!")
        #     elif command == "q":
        #         self.done_recording = True
        
        # b = tfx.pose(transform)
        # b.name = None
        # b = b.as_tf()

        # transformed_pose_data = [b * pose for pose in self.pose_data]

        # transform = tfx.lookupTransform("two_tool_base_link", "two_tool_middle_link")
        # b = tfx.pose(transform)
        # b.name = None
        # b = b.as_tf()

        # transformed_pose_data = [(b * pose) for pose in self.pose_data]

        f = open('pose_data_' + self.arm + '.p', 'wb')
        pickle.dump([dimensions, self.pose_data], f)
        f.close()
        # f = open('pose_data_wrist_' + self.arm + '.p', 'wb')
        # pickle.dump([dimensions, wrist_poses], f)
        # f.close()
        
        self.calculate(dimensions)
        # self.pose_data = wrist_poses
        # self.calculate(dimensions)

        rospy.spin()

    def open_from_file(self):
        # pose_topic = "dvrk_psm1/joint_position_cartesian"
        # rospy.Subscriber(pose_topic, PoseStamped, self.record_pose_callback)   
        rospy.init_node('robo_registration', anonymous=True)
        rospy.Subscriber("/BC/chessboard_pose", PoseStamped, self.get_transform_callback)

        save_camera_transform = raw_input("Do you want to save a new camera transform? (yes/no) ")
        if save_camera_transform == "yes":
            while not self.have_camera_transform and not rospy.is_shutdown():
                print("Waiting for camera transform")
                rospy.sleep(0.5)
            print("Saving camera transform in file")
            f = open("/home/davinci2/catkin_ws/src/davinci_vision/launch/BC_registration/camera_frame.p", "wb")
            pickle.dump(self.camera_transform, f)
            f.close()
        else:
            f = open("/home/davinci2/catkin_ws/src/davinci_vision/launch/BC_registration/camera_frame.p", "rb")
            self.camera_transform = pickle.load(f)
            f.close()

        # f = open('pose_data_transform_' + self.arm + '.p', 'rb')
        # test = pickle.load(f)
        # f.close()
        # print(test)
        # import IPython; IPython.embed()
        # transform = tfx.lookupTransform("two_tool_wrist_sca_shaft_link", "two_remote_center_link")
        f = open('pose_data_' + self.arm + '.p', 'rb')
        data = pickle.load(f)
        dimensions = data[0]
        self.pose_data = data[1]
        print(self.pose_data)
        print(dimensions)
        f.close()

        # transform = tfx.lookupTransform("two_tool_base_link", "two_tool_middle_link")
        # b = tfx.pose(transform)
        # b.name = None
        # b = b.as_tf()

        # trans_data = [tfx.pose(tfx.pose(pose).as_tf() * b).msg.PoseStamped() for pose in self.pose_data]


        # b = tfx.pose(transform)
        # b.name = None
        # b = b.as_tf()

        # transformed_pose_data = [b * pose for pose in self.pose_data]
        # transformed_pose_data = [pose.msg.PoseStamped() for pose in transformed_pose_data]

        # self.pose_data = transformed_pose_data
        





        # self.pose_data = []
        


        # a = "a"
        # while a == "a":
        #     # wrist_poses.append(tfx.lookupTransform("two_tool_wrist_sca_shaft_link", "two_remote_center_link").msg.PoseStamped())
        #     # wrist_poses.append(tfx.lookupTransform("two_tool_middle_link", "two_remote_center_link").msg.PoseStamped())
        #     # tfx.lookupTransform("two_tool_middle_link", "two_remote_center_link")
        #     # tfx.lookupTransform("two_tool_middle_link", "two_remote_center_link")
        #     self.recording_pose = True
        #     # wrist_poses.append(tfx.lookupTransform("two_remote_center_link", "world").msg.PoseStamped())
        #     # print(tfx.lookupTransform("two_tool_wrist_sca_shaft_link", "two_remote_center_link"))
        #     # print(tfx.lookupTransform("two_tool_wrist_sca_shaft_link", "two_remote_center_link").msg.PoseStamped())
        #     a = raw_input("hi")

        # # trans_data = [(b * pose).msg.PoseStamped() for pose in self.pose_data]
        # offset = tfx.transform([0, 0, -0.00483], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        # trans_data = [tfx.pose(tfx.pose(pose).as_tf() * offset).msg.PoseStamped() for pose in self.pose_data]
        # import IPython; IPython.embed()
        # plt3d = plt.figure().gca(projection='3d')
        # x = []
        # y = []
        # z = []
        # for pose in trans_data:
        #     x.append(pose.pose.position.x)
        #     y.append(pose.pose.position.y)
        #     z.append(pose.pose.position.z)
        # x = np.array(x)
        # y = np.array(y)
        # z = np.array(z)
        # data = np.concatenate((x[:, np.newaxis], y[:, np.newaxis], z[:, np.newaxis]), axis=1)
        # plt3d.scatter3D(*data.T, c='red')
        # x1 = []
        # y1 = []
        # z1 = []
        # for pose in self.pose_data:
        #     x1.append(pose.pose.position.x)
        #     y1.append(pose.pose.position.y)
        #     z1.append(pose.pose.position.z)
        # x1 = np.array(x1)
        # y1 = np.array(y1)
        # z1 = np.array(z1)
        # data1 = np.concatenate((x1[:, np.newaxis], y1[:, np.newaxis], z1[:, np.newaxis]), axis=1)
        # plt3d.scatter3D(*data1.T, c='blue')
        # # for pose in self.pose_data:
        # #     plt3d.plot(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        # plt.show()
        # import IPython; IPython.embed()

        self.calculate(dimensions)

    def calculate(self, dimensions):
        x = []
        y = []
        z = []
        for pose in self.pose_data:
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)
            z.append(pose.pose.position.z)
        # self.pose_data = [[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] for pose in self.pose_data]
        point, normal = self.plane_fit([x, y, z])
        print(point)
        print(normal)
        d = -point.dot(normal)
        xyz = np.array([x, y, z])
        p0 = list(normal)
        p0.append(d)
        
        def f_min(X, p):
            plane_xyz = p[0:3]
            distance = (plane_xyz*X.T).sum(axis=1) + p[3]
            return distance / np.linalg.norm(plane_xyz)

        def residuals(params, signal, X):
            return f_min(X, params)


        sol = leastsq(residuals, p0, args=(None, xyz))[0]

        print "Old solution: ", p0
        print "Solution: ", sol
        print "Old Error: ", (f_min(xyz, p0)**2).sum()
        print "New Error: ", (f_min(xyz, sol)**2).sum()
        self.plot(point, normal, min(x) - 0.1, max(x) + 0.1, min(y) - 0.1, max(y) + 0.1)

        # oh fit_3d_line_pca already gives a unit vector
        cols = dimensions[0]
        rows = dimensions[1]
        row_vectors = []
        col_vectors = []
        for row in range(rows):
            row_vectors.append(self.fit_3d_line_pca([i + (row*cols) for i in reversed(range(cols))], self.pose_data)[0])
        for col in range(cols):
            col_vectors.append(self.fit_3d_line_pca([col + cols * i for i in range(rows)], self.pose_data)[0])
        row_vector = self.fit_3d_line_pca(range(cols), self.pose_data)[0]
        col_vector = self.fit_3d_line_pca([cols * i for i in range(rows)], self.pose_data)[0]
        mean_row_vector = [0, 0, 0]
        for vector in row_vectors:
            mean_row_vector[0] += vector[0]
            mean_row_vector[1] += vector[1]
            mean_row_vector[2] += vector[2]
        mean_row_vector[0] /= len(row_vectors)
        mean_row_vector[1] /= len(row_vectors)
        mean_row_vector[2] /= len(row_vectors)
        mean_row_vector = np.array(mean_row_vector)
        print("meanrow " + str(mean_row_vector))
        mean_row_vector /= np.linalg.norm(mean_row_vector)

        mean_col_vector = [0, 0, 0]
        for vector in col_vectors:
            mean_col_vector[0] += vector[0]
            mean_col_vector[1] += vector[1]
            mean_col_vector[2] += vector[2]
        mean_col_vector[0] /= len(col_vectors)
        mean_col_vector[1] /= len(col_vectors)
        mean_col_vector[2] /= len(col_vectors)
        print("meancol " + str(mean_col_vector))
        mean_col_vector /= np.linalg.norm(mean_col_vector)

        cross = np.cross(row_vector, col_vector)

        meancross = np.cross(mean_row_vector, mean_col_vector)
        meancross /= np.linalg.norm(meancross)
        meancolcross = np.cross(meancross, mean_row_vector)
        meancolcross /= np.linalg.norm(meancolcross)
        print("row " + str(row_vector))
        print("meanrow " + str(mean_row_vector))
        print("col " + str(col_vector))
        print("meancol " + str(mean_col_vector))
        print("meancolcross " + str(meancolcross))
        print("cross " + str(cross))
        print("meancross " + str(meancross))
        print("compare to norm " + str(normal))
        x = [mean_row_vector[0], meancolcross[0], meancross[0]]
        y = [mean_row_vector[1], meancolcross[1], meancross[1]]
        z = [mean_row_vector[2], meancolcross[2], meancross[2]]
        print(x)
        print(y)
        print(z)
        x = np.array(x)
        y = np.array(y)
        z = np.array(z)
        data = np.concatenate((x[:, np.newaxis], y[:, np.newaxis], z[:, np.newaxis]), axis=1)
        #okay so dot product between meanrow & meancol is 0.0159.
        plt3d = plt.figure().gca(projection='3d')
        plt3d.scatter3D(*data.T, c=['red', 'green', 'blue'])



        # for pose in self.pose_data:
        #     plt3d.plot(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        # plt.axis([0, 1000, 0, 12000])
        plt.show()
        xa = mean_row_vector
        # xa = -xa
        ya = meancolcross
        # ya = -ya
        za = meancross
        # print("normalized norm " + str(normal/np.linalg.norm(normal)))
        # I'll just take the average of the rows and average of the columns...
        transform = tfx.pose(point, [[xa[0], ya[0], za[0]],[xa[1], ya[1], za[1]],[xa[2], ya[2], za[2]]])
        # transform.translation.z += CAP_OFFSET

        f = open("/home/davinci2/catkin_ws/src/davinci_vision/launch/BC_registration/robot_transform_" + self.arm +\
            ".p", "wb")
        pickle.dump(transform, f)
        f.close()
        camera_transform = tfx.pose(self.camera_transform)
        # print("CAM TRANS")
        # print(camera_transform)
        
        # print("NEW CAM TRANS")
        print(camera_transform)
        final_transform = tfx.inverse_tf(camera_transform).as_transform() * transform
        # why doesn't this work......
        # maybe this should be the inverse lol
        # final_transform = tfx.inverse_tf(final_transform)
        print(final_transform)
        pt = final_transform.translation
        rot = final_transform.rotation
        print("x: " + str(pt.x))
        print("y: " + str(pt.y))
        print("z: " + str(pt.z))
        print("x: " + str(rot.x))
        print("y: " + str(rot.y))
        print("z: " + str(rot.z))
        print("w: " + str(rot.w))
        final_transform_str = str(pt.x) + " " + str(pt.y) + " " + str(pt.z) + " " + str(rot.x) + \
            " " + str(rot.y) + " " + str(rot.z) + " " + str(rot.w)
        f = open("/home/davinci2/catkin_ws/src/davinci_vision/launch/BC_registration/transform_" + self.arm +\
            ".txt", "w")
        f.write(final_transform_str)
        f.close()
        print("wrote transform to transform_" + self.arm + ".txt.")


        # take this and multiply with /BC/chessboard and we're done?

    def fit_3d_line_pca(self, indices, pose_data):
        x = []
        y = []
        z = []
        for i in indices:
            pose = pose_data[i].pose.position
            x.append(pose.x)
            y.append(pose.y)
            z.append(pose.z)
        x = np.array(x)
        y = np.array(y)
        z = np.array(z)

        data = np.concatenate((x[:, np.newaxis], y[:, np.newaxis], z[:, np.newaxis]), axis=1)
        datamean = data.mean(axis=0)
        
        uu, dd, vv = np.linalg.svd(data - datamean)

        vein_p1 = np.array(vv[0][1])
        vein_p2 = np.array(vein_p1 + vv[0][0])
        print("point1 " + str(vein_p1))
        print("point2 " + str(vein_p2))

        return [vv[0], datamean, data]#, sum_errors

if __name__ == '__main__':
    r = RoboRegistration(sys.argv[1])
    if sys.argv[2] == "file":
        print("file")
        r.open_from_file()
    else:
        print("listener")
        r.listener()
