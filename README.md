# ros2bagsend
A tool that uses simple logic to send topics recorded in ros2 bag one cycle at a time (not a strict cycle). I use this tool to step-submit and debug rosbag topics synchronously. https://github.com/PINTO0309/simple-ros2-processing-tools

[![Downloads](https://static.pepy.tech/personalized-badge/ros2bagsend?period=total&units=none&left_color=grey&right_color=brightgreen&left_text=Downloads)](https://pepy.tech/project/ros2bagsend)

https://github.com/PINTO0309/ros2bagsend/assets/33194443/b25069a8-300e-4913-bb59-7477b3927561
  
## 1. Install ROS2
```bash
DISTRO=humble

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y \
ros-${DISTRO}-rosbag2 \
ros-${DISTRO}-vision-opencv \
ros-${DISTRO}-vision-msgs \
ros-${DISTRO}-image-pipeline
```
## 2. Install ros2bagsend
```
pip install ros2bagsend
```
## 3. Usage
- Send topics in units of pseudo-frames
  ```bash
  ros2bagsend --ros-args \
  -p bag_file_path:='test.db3'
  ```
  ```
  topic summary ########################################
  topic_info.name0: /camera/aligned_depth_to_color/image_raw
  topic_info.name1: /camera/color/camera_info
  topic_info.name2: /camera/color/image_raw
  topic_info.name3: /zed2i/zed_node/depth/depth_registered
  topic_info.name4: /zed2i/zed_node/rgb/camera_info
  topic_info.name5: /zed2i/zed_node/rgb_raw/image_raw_color
  ######################################################
  Press Enter to publish the next frame... (Interrupted by 'q' key)
  1./zed2i/zed_node/rgb/camera_info: /zed2i/zed_node/rgb/camera_info time_stamp: 1703229424121013788
  1./zed2i/zed_node/rgb_raw/image_raw_color: /zed2i/zed_node/rgb_raw/image_raw_color time_stamp: 1703229424128303140
  Press Enter to publish the next frame... (Interrupted by 'q' key)
  2./zed2i/zed_node/rgb/camera_info: /zed2i/zed_node/rgb/camera_info time_stamp: 1703229424128367120
  2./zed2i/zed_node/depth/depth_registered: /zed2i/zed_node/depth/depth_registered time_stamp: 1703229424134155118
  2./camera/color/camera_info: /camera/color/camera_info time_stamp: 1703229424138749661
  2./camera/aligned_depth_to_color/image_raw: /camera/aligned_depth_to_color/image_raw time_stamp: 1703229424150239208
  2./camera/color/image_raw: /camera/color/image_raw time_stamp: 1703229424154925667
  Press Enter to publish the next frame... (Interrupted by 'q' key)
  3./zed2i/zed_node/rgb/camera_info: /zed2i/zed_node/rgb/camera_info time_stamp: 1703229424184716213
  3./zed2i/zed_node/rgb_raw/image_raw_color: /zed2i/zed_node/rgb_raw/image_raw_color time_stamp: 1703229424191219846
  Press Enter to publish the next frame... (Interrupted by 'q' key)
  4./zed2i/zed_node/rgb/camera_info: /zed2i/zed_node/rgb/camera_info time_stamp: 1703229424191263193
  4./zed2i/zed_node/depth/depth_registered: /zed2i/zed_node/depth/depth_registered time_stamp: 1703229424195592537
  4./camera/aligned_depth_to_color/image_raw: /camera/aligned_depth_to_color/image_raw time_stamp: 1703229424201049523
  4./camera/color/camera_info: /camera/color/camera_info time_stamp: 1703229424201093864
  4./camera/color/image_raw: /camera/color/image_raw time_stamp: 1703229424204340722
  Press Enter to publish the next frame... (Interrupted by 'q' key)
    :
    :
  ```

- Specifies frames to start or end transmission
  `frames` refers to the number of times a topic is sent, not the number of topics. In the example below, topics are sent in the range from frame 5 to frame 100, skipping up to frame 4. It is advisable to check in advance which frame range should be sent using `rqt` or similar.
  ```bash
  ros2bagsend --ros-args \
  -p bag_file_path:='test.db3' \
  -p start_frame_no:=5 \
  -p end_frame_no:=100
  ```

- Only analysis of statistical information is performed
  Analyze the list of topic names in the bag file and the number of assumed frames. It is also possible to specify `start_frame_no` and `end_frame_no`. If `statistical_analysis_only` is specified, no topics are sent.
  ```bash
  ros2bagsend --ros-args \
  -p bag_file_path:='test.db3' \
  -p statistical_analysis_only:=True
  ```
  ```
  Statistical information is being analyzed ...
  topic summary ########################################
  topic_info.name0: /camera/aligned_depth_to_color/image_raw
  topic_info.name1: /camera/color/camera_info
  topic_info.name2: /camera/color/image_raw
  topic_info.name3: /zed2i/zed_node/depth/depth_registered
  topic_info.name4: /zed2i/zed_node/rgb/camera_info
  topic_info.name5: /zed2i/zed_node/rgb_raw/image_raw_color
  ======================================================
  Approximate total frame count: 1,104
  ######################################################
  Done.
  ```
