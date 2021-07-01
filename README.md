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
