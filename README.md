# Aprial_tag

https://github.com/koide3/gazebo_apriltag

https://github.com/AprilRobotics/apriltag-generation

https://github.com/AprilRobotics/apriltag-imgs

https://github.com/YuehChuan/tb3_aprilTag

https://github.com/AprilRobotics/apriltag_ros

https://github.com/sbhatti915/Hospital-Guide-Robot-Mobile-Robotics-Project-


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

roslaunch realsense2_camera rs_camera.launch

realsense-viewer

rosrun rqt_reconfigure rqt_reconfigure
