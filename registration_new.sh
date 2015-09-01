#!/bin/bash
#should probably make some variables
#roslaunch raven_2_vision chessboard.launch width:=9 height:=7 square:=0.0127 image:=/BC/right/image_rect info:=/BC/right/camera_info frame:=right_BC topic:=/BC/right/chessboard_pose &
#roslaunch raven_2_vision chessboard.launch width:=9 height:=7 square:=0.0127 image:=/BC/left/image_rect info:=/BC/left/camera_info frame:=left_BC topic:=/BC/left/chessboard_pose &

roslaunch raven_2_vision chessboard2.launch &
rosrun raven_pose_estimator pose_estimator --left /BC/left --right /BC/right --left_poses /BC/left/chessboard_pose --right_poses /BC/right/chessboard_pose --output /BC/chessboard_pose --frame BC &
#rosrun davinci_vision robo_registration.py left listener
#rosrun davinci_vision robo_registration.py right listener

