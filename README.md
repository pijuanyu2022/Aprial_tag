# Aprial_tag

https://github.com/koide3/gazebo_apriltag

https://github.com/AprilRobotics/apriltag-generation

https://github.com/AprilRobotics/apriltag-imgs

https://github.com/YuehChuan/tb3_aprilTag

https://github.com/AprilRobotics/apriltag_ros

https://github.com/sbhatti915/Hospital-Guide-Robot-Mobile-Robotics-Project-

https://github.com/thien94/vision_to_mavros

Quickstart:

Step1: source ~/catkin_real/devel/setup.bash

Step2: roslaunch realsense2_camera rs_t265.launch

Step3: source ~/catkin_april/devel_isolated/setup.bash

Step4: roslaunch test_pkg  t265_fisheye_undistort.launch

Step5: source ~/catkin_april/devel_isolated/setup.bash

Step6: roslaunch apriltag_ros continuous_detection_fish1s_d.launch

Step7: source ~/catkin_april/devel_isolated/setup.bash

Step8: roslaunch apriltag_ros continuous_detection_fish2s_d.launch

Step9: rqt_image_view

Step10: roslaunch rtabmap_ros rtabmap.launch    args:="-d --Rtabmap/ImagesAlreadyRectified true --Mem/UseOdomGravity true --Optimizer/GravitySigma 0.3"    stereo:=true    left_image_topic:=/camera/fisheye1/rect/mage   right_image_topic:=/camera/fisheye2/rect/image_raw    left_camera_info_topic:=/camera/fisheye1/rect/camera_info    right_camera_info_topic:=/camera/fisheye2/rect/camera_info    visual_odometry:=false    odom_frame_id:=camera_odom_frame

-------------
-------------
-------------







APRIL TAG 

Step1: roscore

Step2: source ~/catkin_april/devel_isolated/setup.bash

Step3: roslaunch apriltag_ros continuous_detection.launch

Step4: rqt_image_view

Step5: select "/tag_detections_image"

Step6: rostopic echo /tf

Step7: rostopic hz /camera/infra1/image_rect_raw


USB: 


Step1: source ~/catkin_web/devel/setup.bash

Step2: roslaunch usb_cam usb_cam-test.launch

Step3: rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.023 image:=/usb_cam/image_raw camera:=/usb_cam


Real Sense
https://roboticslab-uc3m.github.io/installation-guides/install-realsense2.html

Step1: source ~/catkin_real/devel/setup.bash

roslaunch realsense2_camera rs_camera.launch

roslaunch realsense2_camera rs_t265.launch


realsense-viewer

rosrun rqt_reconfigure rqt_reconfigure

ssh omnid@omnid1




