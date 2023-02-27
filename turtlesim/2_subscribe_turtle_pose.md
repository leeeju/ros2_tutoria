## rclpy_tutorial/ turtlesim/2_subscribe_turtle_pose

##  

**출처 :**  

**튜토리얼 레벨 :**  초급

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy



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







