### ROS2 (Foxy Fitzroy) 설치 및 개발환경 설정

참조1: <https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html>

참조2:<https://cafe.naver.com/openrt/25288>

#### ROS2 설치(바이너리 패키지 설치)

**선행 작업: Ubuntu 20.04(Focal Fossa) LTS 설치**

ROS Foxy Fitzroy 바이너리 패키지는 우분투 버전 중에서는 Ubuntu 20.04(Focal Fossa) 만을 지원한다. 이 후 내용은 이미 Ubuntu 20.04 LTS 버전이 설치된 상태를 전제로 한다.

##### Set **locale**

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```



##### 소스 설정



```
apt-cache policy | grep universe
```





```

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
```





##### 소스설정




```
sudo apt update && sudo apt install curl gnupg2 lsb-release
```




```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
```



```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```





##### ROS2 데비안 패키지 설치




```
sudo apt update
```



```
sudo apt install ros-foxy-desktop ros-foxy-rmw-fastrtps* ros-foxy-rmw-cyclonedds*
```



ROS2 삭제

```
sudo apt remove ~nros-foxy-* && sudo apt autoremove
```



##### ROS2 패키지 설치 확인

 `talker` 퍼블리셔 노드와 `listener` 서브스크라이버 노드를 구동하여ROS2 패키지 설치를 확인해 보자.

이를 위해우선 ROS 2 환경 설정을 위한 설정 스크립트를 소싱한다.

```
source /opt/ros/foxy/setup.bash
```

`talker` 퍼블리셔 노드 실행

```
ros2 run demo_nodes_cpp talker
[INFO] [1612912263.574031946] [talker]: Publishing: 'Hello World: 1'
[INFO] [1612912264.574010597] [talker]: Publishing: 'Hello World: 2'
[INFO] [1612912265.574381893] [talker]: Publishing: 'Hello World: 3'
[INFO] [1612912266.574508130] [talker]: Publishing: 'Hello World: 4'
[INFO] [1612912267.574615200] [talker]: Publishing: 'Hello World: 5'
[INFO] [1612912268.574767202] [talker]: Publishing: 'Hello World: 6'
[INFO] [1612912269.574953419] [talker]: Publishing: 'Hello World: 7'
...
```

다른 터미널 창을 열고 역시 ROS 2 환경 설정을 위한 설정 스크립트를 소싱한 후, 

```
source /opt/ros/foxy/setup.bash
```

```listener``` 서브스크라이버 노드를 구동하여 동작을 확인한다.

```
ros2 run demo_nodes_py listener
[INFO] [1612912265.593335793] [listener]: I heard: [Hello World: 3]
[INFO] [1612912266.576514520] [listener]: I heard: [Hello World: 4]
[INFO] [1612912267.576780341] [listener]: I heard: [Hello World: 5]
[INFO] [1612912268.576769156] [listener]: I heard: [Hello World: 6]
[INFO] [1612912269.577142775] [listener]: I heard: [Hello World: 7]
```



##### ROS2 개발 툴 설치

ROS2 로봇 프로그래밍에 필요한 소프트웨어 들을 설치한다.


```
sudo apt update && sudo apt install -y \
build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
```




```
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
```




```
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev
```







#### ROS2 개발환경 설정

##### 워크스페이스 폴더 생성

```
mkdir -p ~/robot_ws/src
```

워크스페이스 폴더의 내용 확인

```
ls ~/robot_ws
src

```

##### 빌드 테스트

빌드를 위해 워크스페이스로 경로 변경

```
cd ~/robot_ws
```



ROS 2 환경 설정을 위한 설정 스크립트 소싱

```
source /opt/ros/foxy/setup.bash
```

빌드

```
colcon build --symlink-install                     
Summary: 0 packages finished [0.17s]
```



빌드 후, 워크스페이스 내용 확인

```
ls
build  install  log  src
```



빌드 후, `build` , `install` , `log` 폴더가 추가된 것을 확인할 수 있다. 



Run Commands 설정(`~/.bashrc` 파일에 추가할 내용)

```
source /opt/ros/foxy/setup.bash
source ~/robot_ws/install/local_setup.bash

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/vcstool-completion/vcs.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/robot_ws

export ROS_DOMAIN_ID=7
export ROS_NAMESPACE=robot1

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_connext_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_gurumdds_cpp

# export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})'
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1

alias cw='cd ~/robot_ws'
alias cs='cd ~/robot_ws/src'
alias ccd='colcon_cd'

alias cb='cd ~/robot_ws && colcon build --symlink-install'
alias cbs='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias cbu='colcon build --symlink-install --packages-up-to'
alias ct='colcon test'
alias ctp='colcon test --packages-select'
alias ctr='colcon test-result'

alias rt='ros2 topic list'
alias re='ros2 topic echo'
alias rn='ros2 node list'

alias killgazebo='killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient'

alias af='ament_flake8'
alias ac='ament_cpplint'

alias testpub='ros2 run demo_nodes_cpp talker'
alias testsub='ros2 run demo_nodes_cpp listener'
alias testpubimg='ros2 run image_tools cam2image'
alias testsubimg='ros2 run image_tools showimage'
```





[튜토리얼 목록](./README.md) 

