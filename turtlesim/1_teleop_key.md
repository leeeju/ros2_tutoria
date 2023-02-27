## rclpy_tutorial/ turtlesim/1_teleop_key

##  

**출처 :**  <https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html>

**튜토리얼 레벨 :**  초급

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy



#### `turtlesim`  노드의 거북이를 제어하는 노드 작성을 위해  `turtle_pkg`패키지를 만들고 키보드로 원격 조종하는 노드를 작성해보자. `

##### 1. 사전 조사. 

어떤 형식의 무슨 토픽을 발행해야 하는 가를 조사하기 위해 `turtlesim` 패키지의 `turtlesim_node` 와 `turtle_teleop_key` 노드를 실행 후, `rqt_graph`를 실행한다. 

```
ros2 run turtlesim turtlesim_node
```



```
ros2 run turtlesim turtle_teleop_key
```



```
rqt_graph
```

`rqt_graph`를 실행하면 아래와 같은 그해프를 볼 수 있다. 이 그래프가 나타내는 것은 `teleop_turtle` 노드가 `/turtle1/cmd_vel` 토픽을 발행 중이며, `turtlesim`노드가 해당 토픽을 구독 중이라는 것이다. 따라서 우리가 작성해야 할 노드는 토픽 퍼블리셔이며 발행할 토픽명은 `/turtle1/cmd_vel`이다. 



<img src="./img/teleop_rqt_graph.png" style="zoom:80%;" >

이제 필요한 정보는 `turtle1/cmd_vel` 토픽이 어떤 토픽인가 이다. 이를 알아내기 위해 `ros2 topic type` 명령을 이용하자. 

```
ros2 topic type /turtle1/cmd_vel
geometry_msgs/msg/Twist
```

거북이를 제어하는 `/turtle1/cmd_vel`토픽의 형식이 `geometry_msgs/msg/Twist`형식이라는 것을 알아냈다. 웹 브라우저에서 `geometry_msgs/msg/Twist` 를 검색해 보면 아래와 같은 결과를 얻을 수 있다. 

<img src="./img/geometry_msgs_twist_msg.png" width=1405  >

해석하자면 `Twist` 메세지는 `Vector3` 형식의 `linear`메세지와 와 역시 `vector3` 형식의 `angular` 메세지가 합쳐진 형식이다. `vector3`를 검색해 보면 다음과 같은 결과를 찾을 수 있었다.

<img src="./img/geometry_msgs_vector3_msg.png" style="zoom:;" >

`ros2 topic echo /turtle1/cmd_vel` 명령을 실행 후 



`turtle_teleop_key` 노드를 실행한 창에서 `↑` , `↓`,   `←` , `→`키를 눌러 보면 `ros2 topic echo /turtle1/cmd_vel` 명령을 실행한 창에서 다음과 같은 반응을 관찰할 수 있다.

```
ros2 topic echo /turtle1/cmd_vel 
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
linear:
  x: -2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 2.0
---
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -2.0
```

정리하자면  `↑` 키가 눌리면 `linear:x`의 값이 `2.0`이되고, 거북이는 전진하고,

​                     `↓`키가 눌리면 `linear:x`의 값이 `-2.0`이되고, 거북이는 후진하며, , 

  `←` 키가 눌리면 `angular:z`의 값이 `2.0`이되고, 거북이는 좌로 회전하고,

 `→`키가 눌리면 `angular:z`의 값이 `-2.0`이되고, 거북이는 우로 회전한다. 이 사실들을 바탕으로 `↑` , `↓`,   `←` , `→`키를 각각 `w` , `s`,   `a` , `d`키로 바꾼 `remote_turtle` 노드를 작성해보자. 이 외에도 `turtlesim`노드의 거북이를 제어하는 코드를 추가할 계획이므로 패키지 이름은 `turtle_pkg`로 정하도록 하자.

*작업* 경로를 워크스페이스`~/robot_ws` 의 `src` 폴더로 변경한다. 

```
cd ~/robot_ws/src
```

`rclpy` 와 `geometry_msgs` 에 의존성을 갖는 패키지 `turtle_pkg` 생성

```
ros2 pkg create turtle_pkg --build-type ament_python --dependencies rclpy geometry_msgs
```

`~/robot_ws/src/turtle_pkg/turtle_pkg`폴더로 작업 경로 변경

```
cd ~/robot_ws/src/turtle_pkg/turtle_pkg
```

`ls` 명령으로 작업경로에 `__init__.py`파일의 존재를 확인한다.

```
ls__init__.py
```

`__init__.py`파일과 같은 경로에 키보드 입력을 받는  `getchar.py`를 작성한다. (이 코드는 ROS2 노드는 아니다. 키보드 입력을 받기위한 일종의 사용자 라이브러리 이다.)

```
gedit getchar.py &
```

```python
import os, time, sys, termios, atexit, tty
from select import select
  
# class for checking keyboard input
class Getchar:
    def __init__(self):
        # Save the terminal settings
        self.fd = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.fd)
        self.old_term = termios.tcgetattr(self.fd)
  
        # New terminal setting unbuffered
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)
  
        # Support normal-terminal reset at exit
        atexit.register(self.set_normal_term)      
      
    def set_normal_term(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)
  
    def getch(self):        # get 1 byte from stdin
        """ Returns a keyboard character after getch() has been called """
        return sys.stdin.read(1)
  
    def chk_stdin(self):    # check keyboard input
        """ Returns True if keyboard character was hit, False otherwise. """
        dr, dw, de = select([sys.stdin], [], [], 0)
        return dr
```





##### `turtlsim`노드의 거북이 원격 조종 노드 작성

`turtlsim`노드의 거북이를 제어하는 몇가지 코드를 작성할 `script` 폴더를 만들고  경로를 해당 폴더로 변경한다. 

```
mkdir script
cd script
```

키보드로 `turtlsim`노드의 거북이를 제어하는 `remote_turtle` 노드를 작성한다. 

```
gedit  remote_turtle.py &
```

```python
import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from turtle_pkg.getchar import Getchar

msg = """          forward
              +---+
              | w |
          +---+---+---+
turn left | a | s | d | turn left
          +---+---+---+
           backward
### space for stop\n
"""

class RemoteTurtle(Node):

    def __init__(self):
        self.cnt_sec = 0
        super().__init__('remote_turtle')
        qos_profile = QoSProfile(depth=10)
        #self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile)
        self.timer    = self.create_timer(1, self.count_sec)
        #self.subscription  # prevent unused variable warning

    def get_pose(self, msg):
        self.pose = msg
        #self.get_logger().info('x = "%s", y="%s", theta="%s"' %(self.pose.x, self.pose.y, self.pose.theta))

    def count_sec(self):
        self.cnt_sec = self.cnt_sec + 1
        #print(self.cnt_sec)


def main(args=None):
    rclpy.init(args=args)
    node= RemoteTurtle()
    pub = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    tw = Twist()
    kb = Getchar()
    key = ' '
    count = 0
    print(msg)
    try:
            while rclpy.ok():
                key = kb.getch()
                if      key == 'w':
                    print("forward")
                    count = count + 1
                    tw.linear.x  =  tw.angular.z =  0.0
                    tw.linear.x  =  2.0
                elif key == 's':
                    print("backward")
                    count = count + 1
                    tw.linear.x  =  tw.angular.z =  0.0
                    tw.linear.x  = -2.0
                elif key == 'a':
                    print("turn left")
                    count = count + 1
                    tw.linear.x  =  tw.angular.z =  0.0
                    tw.angular.z =  2.0
                elif key == 'd':
                    print("turn right")
                    count = count + 1
                    tw.linear.x  =  tw.angular.z =  0.0
                    tw.angular.z = -2.0
                elif key == ' ':
                    count = count + 1
                    print("stop")
                    tw.linear.x  =  tw.angular.z =  0.0
                pub.publish(tw)
                count = count % 15
                if count == 0:
                    print(msg)
                #rclpy.spin_once(node, timeout_sec=0.1)
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
    maintainer='gnd1',
    maintainer_email='02stu4@gmail.com',
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
'remote_turtle   = turtle_pkg.script.remote_turtle:main',
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

`remote_turtle` 노드를 구동하려 `turtlesim` 노드의 거북이가 조종되는 가를 확인한다. 

```
ros2 run turtle_pkg remote_turtle 
           forward
              +---+
              | w |
          +---+---+---+
turn left | a | s | d | turn left
          +---+---+---+
             backward
### space for stop
```















[튜토리얼 목록](../README.md) 







