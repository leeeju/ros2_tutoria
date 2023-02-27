## rclpy_tutorial/ turtlebot3/howto use raspicam

##  

**출처 :**  <https://github.com/christianrauch/raspicam2_node>

**튜토리얼 레벨 :**  초급

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy

`ssh`를 이용해 터틀봇3의 라즈베리파이에 연결한다.(`id`: `ubuntu`, `password`: `turtlebot`)



```
ubuntu@10.42.0.xxx
```

라즈베리파이의 카메라 인터페이스 활성화를 위해 `/boot/firmware/config.txt` 파일을 편집한다.

```
sudo nano /boot/firmware/config.txt
```

`/boot/firmware/config.txt` 파일 하단에 다음 2줄의 내용을 추가한다. 

```
start_x = 1
gpu_mem = 128
```

`/boot/firmware/config.txt`

`cv_bridge` 설치 

```
sudo apt install ros-foxy-cv-bridge
```

VideoCore 라이브러리를 설치를 위해 이와 충돌하는 이미 설치되어 있는 `mesa-common-dev`를 삭제한다. 

```
sudo apt autoremove --purge libgles2-mesa-dev mesa-common-dev
```

`image-common` 설치

```
sudo add-apt-repository ppa:ubuntu-pi-flavour-makers/ppa
 Assorted software and drivers to support the creation of community created Ubuntu images for the Raspberry Pi  2, 3 & 3+
 More info: https://launchpad.net/~ubuntu-pi-flavour-makers/+archive/ubuntu/ppa
Press [ENTER] to continue or Ctrl-c to cancel adding it.
```

`v4l2-camera` 설치



```
sudo apt install ros-foxy-ros-foxy-v4l2-camera
```







```
ubuntu@10.42.0.xxx
```

`/boot/firmware/config.txt` 파일 편집

```
sudo nano /boot/firmware/config.txt
```

`/boot/firmware/config.txt` 파일 하단에 다음 2줄의 내용을 추가한다. 

```
start_x = 1
gpu_mem = 128
```

`/boot/firmware/config.txt`



```
# Please DO NOT modify this file; if you need to modify the boot config, the
# "usercfg.txt" file is the place to include user changes. Please refer to
# the README file for a description of the various configuration files on
# the boot partition.

# The unusual ordering below is deliberate; older firmwares (in particular the
# version initially shipped with bionic) don't understand the conditional
# [sections] below and simply ignore them. The Pi4 doesn't boot at all with
# firmwares this old so it's safe to place at the top. Of the Pi2 and Pi3, the
# Pi3 uboot happens to work happily on the Pi2, so it needs to go at the bottom
# to support old firmwares.

[pi4]
kernel=uboot_rpi_4.bin
max_framebuffers=2

[pi2]
kernel=uboot_rpi_2.bin

[pi3]
kernel=uboot_rpi_3.bin

[all]
arm_64bit=1
device_tree_address=0x03000000

# The following settings are "defaults" expected to be overridden by the
# included configuration. The only reason they are included is, again, to
# support old firmwares which don't understand the "include" command.

enable_uart=1
cmdline=cmdline.txt

include syscfg.txt
include usercfg.txt
# add bellow for using pi camera module

start_x = 1
gpu_mem = 128



```

사용하려고하는 `v4l2_camera` 노드는 기본적으로 `/dev/video0` 장치를 사용한다.

`/dev/video0` 장치 확인



```
ls /dev/vid*
/dev/vid*
/dev/video10  /dev/video12  /dev/video14  /dev/video16
/dev/video11  /dev/video13  /dev/video15

```

`/dev/video0` 장치가 존재하지  않는 것을 확인.

라즈제리파이 카메라의`장치명 /dev/video0` 로의 연결을 위해 `/etc/modules` 





#### `turtlesim`  노드의 거북이의 `pose`토픽을 구독하는 노드를 이미 만들어 둔  `turtle_pkg`패키지에 추가한다.

##### 1. `turtlesim`노드의 거북이는 가로, 세로 각각 (0,0)~(11.0,11.0)의 네모난 세상에서 살고 있다. 거북이를 임의의 좌표로 이동시키려면 현재 거북이의 위치를 알아내야 한다. 해당 정보는 `/turtle1/pose`토픽으로 발행되고 있다. 

`turtlesim` 패키지의 `turtlesim_node` 와 `turtle_teleop_key` 노드를 실행 후, `/turtle1/pose`토픽을 `echo` 시킨다. 

```
ros2 run turtlesim turtlesim_node
```



```
ros2 topic echo /turtle1/pose 
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
```



`turtle_teleop_key` 노드를 실행하여, 거북이를 이곳, 저곳으로 이동 시키며 `echo`시킨 `pose` 토픽의 변화를 살펴보자

```
ros2 run turtlesim turtle_teleop_key
```

`ros2 topic type` 명령으로 `/turtle1/pose`토픽의 형식을 알아보자

```
ros2 topic type /turtle1/pose 
turtlesim/msg/Pose
```

`turtlesim`거북이의 `pose`토픽 구독노드 `sub_turtle_pose.py`작성을 위해 작업 경로를 변경한다.

```
cd ~/robot_ws/src/turtle_pkg/turtle_pkg/script
```



 `sub_turtle_pose.py`작성



```
gedit sub_turtle_pose.py &
```

```python
import rclpy,sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from turtlesim.msg import Pose
from math import degrees, radians, sqrt, sin, cos, pi


class Turtle_Pose(Node):

    def __init__(self):
        self.pose = Pose()
        self.cnt_sec = 0
        super().__init__('sub_turtle_pose')
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.get_pose, qos_profile )
        self.timer    = self.create_timer(1, self.count_sec)
        self.subscription  # prevent unused variable warning

    def get_pose(self, msg):
        self.pose = msg
        #self.get_logger().info('x = "%s", y="%s", theta="%s"' %(self.pose.x, self.pose.y, self.pose.theta))

    def count_sec(self):
        self.cnt_sec = self.cnt_sec + 1

def main(args=None):
    rclpy.init(args=args)
    node= Turtle_Pose()
    try:
            while rclpy.ok():  
                ### =============================================================================
                print('x = "%s", y="%s", theta="%s(deg)' %(node.pose.x, node.pose.y,round(degrees(node.pose.theta),2)))
        
                duration = node.cnt_sec + 1
        
                while node.cnt_sec < duration: 
                    pass#print(duration - node.cnt_sec)               
                    rclpy.spin_once(node, timeout_sec=1.0)
                ### ==============================================================================
            sys.exit(1)
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
```



`setup.py` 파일 편집을 위해 경로를 `~/robot_ws/src/turtle_pkg`로 변경한다. 

```
cd ~/robot_ws/src/turtle_pkg
```



`setup.py` 파일 편집

```
gedit setup.py &
```



```python
from setuptools import find_packages
from setuptools import setup

package_name = 'turtle_pkg'

setup(
    name=package_name,  
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gnd0',
    maintainer_email='greattoe@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'remote_turtle   = turtle_pkg.script.remote_turtle:main',
        ],
    },
)
```

`entry_points` 필드의 `console_scripts'` 항목에 다음 내용을 추가한다.



```python
'sub_turtle_pose   = turtle_pkg.script.sub_turtle_pose:main',
```



패키지 빌드를 위해 작업 경로를 `~/robot_ws`로 변경한다.

```
cd ~/robot_ws
```

빌드

```
colcon build --symlink-install
```

새로 빌드한 패키지 정보 반영을 위해 다음 명령을 실행한다.

```
. instal/local_setup.bash
```

`sub_turtle_pose` 노드를 구동하여 `turtlesim` 노드의 거북이의 `pose`가 출력되는 지를 확인한다. 

```
ros2 run turtle_pkg sub_turtle_pose 
x = "0.0", y="0.0", theta="0.0(deg)
x = "5.544444561004639", y="5.544444561004639", theta="163.64(deg)
x = "5.544444561004639", y="5.544444561004639", theta="163.64(deg)
x = "5.544444561004639", y="5.544444561004639", theta="163.64(deg)
```















[튜토리얼 목록](../README.md) 







