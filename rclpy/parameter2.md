### rclpy Params Tutorial – Get and Set ROS2 Params with Python

[간단한 Parameter 사용 예제](./rclpy/parameter1.md) 에서 간단한 파라미터 사용법을 알아보았다. 이번 튜토리얼에서는 파이썬 코드로 다른 노드에서 생성한 파라미터에 접근하는 방법을 알아보자. [이전 튜토리얼](./parameter1.md) 에서작성한 `tuetle_by_param` 노드를 실행한다.

```bash
ros2 run param_tutorial turtle_by_param
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle stop!
```



터미널 창을 새로 열어 `ros2 param list` 명령을 실행한다. 



```
ros2 param list
/move_by_param:
  go_turtle
  use_sim_time
```



`move_by_param` 노드의 `go_turtle` 파라미터의 값을 변경하는 새로운 노드 `req_set_param.py`를 작성하려 한다. 그 전에 `ros service list` 명령을 실행하고, 그 결과를 확인해보자.

```
ros service list
/move_by_param/describe_parameters
/move_by_param/get_parameter_types
/move_by_param/get_parameters
/move_by_param/list_parameters
/move_by_param/set_parameters
/move_by_param/set_parameters_atomically
```

여러가지 서비스 서버가 실행 중이다. 결론 부터 말하자면 다른 노드에서 선언된 파라미터의 값을 가져오거나( `get` ) 	변경하기( `set` ) 위해서는 위 서비스 목록의 적당한 서비스를 이용해야만 한다. 어떤 서비스를 이용할 것인 지는 서비스명에서 유추해 볼 수 있다. `move_by_param` 노드의 `go_turtle` 파라미터의 값을 변경하는 새로운 노드 `req_set_param.py`를 작성하려면 `/move_by_param/set_parameters` 서비스를 이용해야 한다. 다시 말해서 `/move_by_param/set_parameters` 서비스를 요청( `request` )하는 서비스 클라이언트 코드를 작성해야 한다. `ros2 service type` 명령으로 

`/move_by_param/set_parameters` 서비스의 형식을 알아내보자.



```
ros2 service type /move_by_param/set_parameters
rcl_interfaces/srv/SetParameters
```



`/move_by_param/set_parameters` 서비스의 형식이 `rcl_interfaces/srv/SetParameters` 이라는 것을 알아내었다. 그렇다는 것은 `/opt/ros/foxy/share/rcl_interfaces/srv/SetParameters.srv` 파일에 서비스 형식이 기술되어 있다는 뜻이다. 다음은 `/opt/ros/foxy/share/rcl_interfaces/srv/SetParameters.srv` 파일의 내용이다. 



```python
# A list of parameters to set.
Parameter[] parameters

---
# Indicates whether setting each parameter succeeded or not and why.
SetParametersResult[] results
```

`---`를 경계로 앞부분이 서비스요청( `request` )이고, 뒷 부분이 그 응답( `response` )에 해당한다. 

이를 바탕으로 `move_by_param` 노드의 `go_turtle` 파라미터의 값을 변경하는 서비스 클라이언트노드 `req_set_param.py`를 작성해 보자 [이전 튜토리얼](./parameter1.md) 에서 만든 `param_tutorial` 패키지에 `req_set_param.py`를 추가하기 위해 작업 경로를 `cd ~/robot_ws/src/param_tutorial/param_tutorial/script` 로 변경한다. 

```
cd ~/robot_ws/src/param_tutorial/param_tutorial/script
```



`req_set_param.py`를 작성한다. 

```
gedit req_set_param.py
```



```python
import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.parameter import Parameter
#from rcl_interfaces.msg import Parameter, ParameterType

SVC_MSG = (sys.argv[1])

class ReqSetParam(Node):
    def __init__(self):
        super().__init__('req_set_param')
        qos_profile = QoSProfile(depth=10)
        self.cli = self.create_client(SetParameters, 'move_by_param/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetParameters.Request()
    def send_request(self):
        self.req.parameters = [Parameter(name='go_turtle', value=SVC_MSG).to_parameter_msg()]
        self.future = self.cli.call_async(self.req)



def main(args=None):
    rclpy.init(args=args)

    client = ReqSetParam()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                print(response)
            except Exception as e:
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```







`setup.py`편집을 위해 작업 경로를 `~/robot_ws/src/param_tutorial`로 변경한다. 



```bash
cd ~/robot_ws/src/param_tutorial
```



`setup.py`를 아래와 같이편집 후, 저장한다. 



```
gedit setup.py
```



```python
from setuptools import find_packages
from setuptools import setup

package_name = 'param_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
                'turtle_by_param = param_tutorial.script.turtle_by_param:main',
                'req_set_param = param_tutorial.script.req_set_param:main',
        ],
    },
)
```



빌드를 위해 작업 경로를 `~/robot_ws`로 변경한다. 



```bash
cd ~/robot_ws
```



다음 명령을 수행하여 `param_tutorial` 패키지를 빌드한다. 



```bash
colcon build --symlink-install --packages-select param_tutorial
```



새로 빌드한 패키지( `param_tutorial` ) 정보 반영을 위해 다음 명령을 실행한다. 



```bash
. install/local_setup.bash
```



`req_set_param.py` 노드를 실행하여 `go_turtle_` 파라미터의 값을 문자열 `'go'`로 바꾸려면 다음과 같이 실행한다. 



```
ros2 run param_tutorial req_set_param go
```

`turtle_by_param`  노드가 실행된 터미널 창의 출력에 다음과 같은 변화가 나타나는 것을 확인한다.



```bash

[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle go!
[INFO]: turtle go!
[INFO]: turtle go!
[INFO]: turtle go!
[INFO]: turtle go!
[INFO]: turtle go!

```



이제`move_by_param` 노드의 `go_turtle` 파라미터의 값을 읽어 화면에 출력하는  `req_get_param.py`를 `param_tutorial` 패키지에 추가해보자.아래는  앞서 `ros2 service list` 명령 실행 결과이다.



```
ros service list
/move_by_param/describe_parameters
/move_by_param/get_parameter_types
/move_by_param/get_parameters
/move_by_param/list_parameters
/move_by_param/set_parameters
/move_by_param/set_parameters_atomically
```

이 번에는 `/move_by_param/get_parameters` 서비스 클라이언트 코드를 작성해야 하므로 `ros2 service type` 명령으로 `/move_by_param/get_parameters` 서비스의 형식을  알아보자.



```
ros2 service type /move_by_param/get_parameters
rcl_interfaces/srv/GetParameters
```

해당 서비스가 `rcl_interfaces/srv/GetParameters` 형식임을 알 수 있다.



다음은 ` /opt/ros/foxy/share/rcl_interfaces/srv/getParameters.srv` 파일의 내용이다.

```python
# TODO(wjwwood): Decide on the rules for grouping, nodes, and parameter "names"
# in general, then link to that.
#
# For more information about parameters and naming rules, see:
# https://design.ros2.org/articles/ros_parameters.html
# https://github.com/ros2/design/pull/241

# A list of parameter names to get.
string[] names

---
# List of values which is the same length and order as the provided names. If a
# parameter was not yet set, the value will have PARAMETER_NOT_SET as the
# type.
ParameterValue[] values
```



이 서비스 파일의 내용 역시 `---` 구분자 이 전의 내용은 서비스 요청() `request` )에 대한 기술이고,  `---` 구분자 이 후의 내용은 서비스 응답( `response` )에 대한 기술이다. 이를 바탕으로`/move_by_param` 노드의  `go_turtle` 파라미터 값을 요청하는 `rcl_interfaces/srv/GetParameters` 서비스 클라이언트 노드 `req_get_param.py` 를 작성하기 위해 작업 경로를 `~/robot_ws/src/param_tutorial/param_tutorial/script` 로 변경한다. 

```
cd ~/robot_ws/src/param_tutorial/param_tutorial/script
```



`req_get_param.py` 를 아래 내용과 같이 작성 후 저장한다. 



```python
import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
#from rcl_interfaces.msg import Parameter, ParameterType


class ReqGetParam(Node):
    def __init__(self):
        super().__init__('req_get_param')
        qos_profile = QoSProfile(depth=10)
        self.cli = self.create_client(GetParameters, 'move_by_param/get_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetParameters.Request()
        self.declare_parameter('go_turtle', 'stop')
    def send_request(self):
        self.req.names = ['go_turtle']
        self.future = self.cli.call_async(self.req)



def main(args=None):
    rclpy.init(args=args)

    client = ReqGetParam()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                print(response.values[0]._string_value)
                # print(Parameter('go_turtle', list, value=response.values))
                
            except Exception as e:
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```





`setup.py`편집을 위해 작업 경로를 `~/robot_ws/src/param_tutorial`로 변경한다. 



```bash
cd ~/robot_ws/src/param_tutorial
```



`setup.py`를 아래와 같이편집 후, 저장한다. 



```
gedit setup.py
```



```python
from setuptools import find_packages
from setuptools import setup

package_name = 'param_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
                'turtle_by_param = param_tutorial.script.turtle_by_param:main',
                'req_set_param = param_tutorial.script.req_set_param:main',
                'req_get_param = param_tutorial.script.req_get_param:main',
        ],
    },
)
```



빌드를 위해 작업 경로를 `~/robot_ws`로 변경한다. 



```bash
cd ~/robot_ws
```



다음 명령을 수행하여 `param_tutorial` 패키지를 빌드한다. 



```bash
colcon build --symlink-install --packages-select param_tutorial
```



새로 빌드한 패키지( `param_tutorial` ) 정보 반영을 위해 다음 명령을 실행한다. 



```bash
. install/local_setup.bash
```



`req_get_param.py` 노드를 실행해보자.



```
ros2 run param_tutorial req_get_param 
stop
```

실행결과 `stop`이 출력 되었다. 이 결과를 `ros2 param get /move_by_param go_turtle` 명령을 실행한 결과와 비교해 보자.

```
ros2 param get /move_by_param go_turtle
String value is: stop
```





