# ROS2

1. ###### 安装

```
sudo apt install ros-<distro>-usb-cam
pip install pydantic==1.10.14
```

2. ###### 查看

```
v4l2-ctl --list-devices

```

3. 启动时切换

```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video1
ros2 launch usb_cam camera.launch.py
```

4. 运行时动态切换

```
ros2 param set /usb_cam video_device /dev/video2

```

5. 确认相机图像话题

```
ros2 topic list

```

6. 查看图像

```
sudo apt install ros-<distro>-rqt-image-view
ros2 run rqt_image_view rqt_image_view
```

7. launch文件（经常切换设备）

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{'video_device': '/dev/video0'}]
        ),
    ])

```

- 自定义usb_cam启动

```
ros2 launch <your_package> usb_cam_launch.py

```

8. 标定

```
sudo apt install ros-humble-camera-calibration
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.025 --ros-args --remap image:=/camera/image_raw --remap camera:=/camera

```

`--size 8x6`：标定板的内部角点数量（8行，6列）。

`--square 0.025`：标定板方格的边长（单位：米）。

`--ros-args --remap image:=/camera/image_raw`：重映射图像话题到你的相机图像话题。

image_height、image_width：图片的高、宽

camera_name：摄像头名

camera_matrix：摄像头的内部参数矩阵

distortion_model：畸变模型

distortion_coefficients：畸变模型的系数

rectification_matrix：为矫正矩阵，一般为单位阵

projection_matrix：为外部世界坐标到像平面的投影矩阵



# ROS1

1. 单目标定

```
sudo apt-get install ros-<distro>-camera-calibration
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _image_width:=640 _image_height:=480 _pixel_format:=yuyv _io_method:=mmap
rosrun camera_calibration cameracalibrator.py --size 7x9 --square 0.025 image:=/usb_cam/image_raw camera:=/usb_cam

```

- `--size 7x9`：标定板的内部角点数量（7行，9列）。
- `--square 0.025`：标定板方格的边长（单位：米）。
- `image:=/usb_cam/image_raw`：相机图像话题名称。
- `camera:=/usb_cam`：相机名称。

- 标定结果通常会保存为一个YAML文件，包含相机内参和畸变参数。你可以将这些参数加载到相机驱动程序中，以校正图像
- launch

```
<launch>
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
    <param name="video_device" value="/dev/video0" />
    <param name="image_width" value="640" />
    <param name="image_height" value="480" />
    <param name="pixel_format" value="yuyv" />
    <param name="io_method" value="mmap" />
  </node>
  
  <node name="cameracalibrator" pkg="camera_calibration" type="cameracalibrator.py" output="screen" args="--size 7x9 --square 0.025 image:=/usb_cam/image_raw camera:=/usb_cam" />
</launch>
```

```
roslaunch <your_package> camera_calibration.launch
```

- ost.txt

```
# oST version 5.0 parameters
描述一个摄像头的内参和外参，以便进行图像的校正和投影

[image]

width   width: 图像的宽度，单位是像素
640

height   height: 图像的高度，单位是像素
480

[narrow_stereo]

camera matrix  (相机矩阵)
441.147220 0.000000 321.594855  
0.000000 406.092110 241.403195
0.000000 0.000000 1.000000

相机矩阵包含了相机的焦距和主点（光学中心）的位置。
𝑓𝑥=441.14722 水平焦距。
𝑓𝑦=406.09211f 垂直焦距。
𝑐𝑥=321.59485 主点的x坐标。
𝑐𝑦=241.40319 主点的y坐标。

distortion   (畸变系数)  径向畸变和切向畸变。
- \( k_1 = 0.143791 \)
- \( k_2 = -0.134370 \)
- \( p_1 = 0.001890 \)
- \( p_2 = 0.000372 \)
- \( k_3 = 0.000000 \)
0.143791 -0.134370 0.001890 0.000372 0.000000

rectification  (校正矩阵) 立体摄像头的校正，使左右摄像头对齐。对于单个摄像头，通常是单位矩阵
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection   (投影矩阵)投影矩阵将3D点投影到2D图像平面。通常在立体视觉中使用，它结合了相机矩阵和外部参数（如旋转和平移）
投影矩阵的形式为：
\[ P = \begin{bmatrix}
f_x' & 0 & c_x' & T_x \\
0 & f_y' & c_y' & T_y \\
0 & 0 & 1 & 0
\end{bmatrix} \]
458.526761 0.000000 321.863208 0.000000
0.000000 421.659437 242.383317 0.000000
0.000000 0.000000 1.000000 0.000000

```

- ost.yaml

```
image_width: 640
image_height: 480
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [441.14722,   0.     , 321.59485,
           0.     , 406.09211, 241.40319,
           0.     ,   0.     ,   1.     ]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.143791, -0.134370, 0.001890, 0.000372, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1., 0., 0.,
         0., 1., 0.,
         0., 0., 1.]
projection_matrix:
  rows: 3
  cols: 4
  data: [458.52676,   0.     , 321.86321,   0.     ,
           0.     , 421.65944, 242.38332,   0.     ,
           0.     ,   0.     ,   1.     ,   0.     ]
```

