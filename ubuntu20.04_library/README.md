README

系统账号：dofbot
系统密码：yahboom

如果需要将代码移植到自己的电脑上运行时,需要将此文件夹下的libdofbot_kinemarics.so动态库放在/usr/lib/目录下.

dofbot_ws和catkin_ws工作空间下的动态库libdofbot_kinemarics.so是在树莓派系统中使用的.

特别注意:此文件夹下的动态库libdofbot_kinemarics.so与dofbot_ws和catkin_ws工作空间内的动态库libdofbot_kinemarics.so,名字相同,使用环境不同.

/usr/lib/ 경로에 .so 파일을 복사해 넣을 것
rosdep 명령으로 설치되어 있던 종속성들을 날리고 새로 설치한 경우, sudo apt-get install ros-noetic-moveit-visual-tools 명령으로 moveit 패키지도 새로 설치해줄 것.
