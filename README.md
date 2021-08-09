# Aprial_tag

https://github.com/koide3/gazebo_apriltag

https://github.com/AprilRobotics/apriltag-generation

https://github.com/AprilRobotics/apriltag-imgs

https://github.com/YuehChuan/tb3_aprilTag

https://github.com/AprilRobotics/apriltag_ros

https://github.com/sbhatti915/Hospital-Guide-Robot-Mobile-Robotics-Project-

https://github.com/thien94/vision_to_mavros

https://github.com/introlab/rtabmap/issues/459

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

Step10: roslaunch rtabmap_ros rtabmap.launch \
   args:="-d --Rtabmap/ImagesAlreadyRectified true --Mem/UseOdomGravity true --Optimizer/GravitySigma 0.3" \
   stereo:=true \
   left_image_topic:=/camera/fisheye1/rect/image \
   right_image_topic:=/camera/fisheye2/rect/image \
   left_camera_info_topic:=/camera/fisheye1/rect/camera_info \
   right_camera_info_topic:=/camera/fisheye2/rect/camera_info \
   visual_odometry:=false \
   odom_frame_id:=camera_odom_frame \
   rviz:=true \
   rtabmapviz:=false

-------------
-------------
$ python camera_info_pub.py \
   _url:=/home/pyu2020/Documents/RTAB-Map/camera_info/t265_left.yaml \
   image:=/camera/fisheye1/image_raw \
   camera_info:=/camera/fisheye1/camera_info_calib
   
   
$ python camera_info_pub.py \
   _url:=/home/pyu2020/Documents/RTAB-Map/camera_info/t265_right.yaml \
   image:=/camera/fisheye2/image_raw \
   camera_info:=/camera/fisheye2/camera_info_calib

-------------
3D image for D435i

Step1: 

source ~/catkin_real/devel/setup.bash

roslaunch realsense2_camera rs_camera.launch \
    align_depth:=true \
    unite_imu_method:="linear_interpolation" \
    enable_gyro:=true \
     enable_accel:=true
     
     
Step2: 

source ~/catkin_april/devel_isolated/setup.bash

rosrun imu_filter_madgwick imu_filter_node \
    _use_mag:=false \
    _publish_tf:=false \
    _world_frame:="enu" \
    /imu/data_raw:=/camera/imu \
    /imu/data:=/rtabmap/imu
    
Step3: 

source ~/catkin_rtab/devel/setup.bash

roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false \
    wait_imu_to_init:=true \
    imu_topic:=/rtabmap/imu \
    rviz:=true \
    rtabmapviz:=false






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




