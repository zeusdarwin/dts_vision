./run.sh run standard infantry_hole true
./run.sh run hero hero true
./run.sh run sentry sentry true

摄像头插好后执行：
  ls -l /dev/v4l/by-path/
pci-0000:06:00.4-usb-0:1:1.0-video-index0 -> ../../video2
pci-0000:07:00.0-usb-0:1.4:1.0-video-index0 -> ../../video4

查看软链接最终指向：
// 左
  readlink -f /dev/v4l/by-path/pci-0000:07:00.0-usb-0:1.4:1.0-video-index0*  
// 右
  readlink -f /dev/v4l/by-path/pci-0000:06:00.4-usb-0:1:1.0-video-index0*    

开usb：
guvcview -d /dev/vidio2


cd /opt/MVS/bin
./MVS.sh
```

ssh连接小电脑：
#以异能者为例
小电脑执行：
先查看ip地址：hostname -I(第一个就行)
自己电脑：
ssh oy@192.168.1.50
进入代码目录：
rsync -avz --progress ./ oy@192.168.1.50:~/dts_vision/
这会把当前目录的代码同步到小电脑的：/home/oy/dts_vision
建议排除不需要传输的目录：

bash /home/darwin/dts_vision/script/rsync.sh dts 192.168.1.117

  先确认自己位于项目根目录：

  pwd
  ls -la

  同步完成后检查：

  ssh oy@192.168.1.50
  cd ~/dts_vision
  ls -la
##若用 usb_cam
ros2 run usb_cam usb_cam_node --ros-args -p image_width:=1280 -p image_height:=720
##image_tools,则为
ros2 run image_tools cam2image --ros-args \
    -p device_id:=4 \
    -p width:=1280 \
    -p height:=720 \
    -p frequency:=30.0

ros2 run camera_calibratioan cameracalibrator --size 7x10 --square 0.0183 image:=/image 

ssh -L 8080:192.168.1.50:9000 oy@192.168.1.50

rsync -avz \
    /home/darwin/dts_vision/config/sentry.yaml \
    oy@192.168.50.2:/home/oy/dts_vision/config/sentry.yaml


rsync -avz \
    /home/darwin/dts_vision/config/infantry_hole.yaml \
    dts@192.168.1.117:/home/dts/dts_vision/config/infantry_hole.yaml


rsync -avz \
    /home/darwin/dts_vision/src/standard.cpp \
    dts@192.168.1.117:/home/dts/dts_vision/src/standard.cpp

rsync -avz \
    /home/darwin/dts_vision/src/sentry.cpp \
    oy@192.168.50.2:/home/oy/dts_vision/src/sentry.cpp

rsync -avz \
    /home/darwin/dts_vision/script/auto_start/sentry.sh \
    oy@192.168.50.2:/home/oy/dts_vision/script/auto_start/sentry.sh


  sudo systemctl stop dts_vision.service
  sudo systemctl disable dts_vision.service

  kill -TERM 2487 2472 2198
```
### 1. 登录并编译
source /opt/ros/humble/setup.bash

cmake -S tools/point_msgs -B build/point_msgs_build \
  -DCMAKE_INSTALL_PREFIX="$PWD/build/point_msgs_install" \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3

cmake --build build/point_msgs_build --target install -j18

source build/point_msgs_install/share/point_msgs/local_setup.bash

cmake -S . -B build \
  -DBUILD_WITH_NAV_ROS2=ON \
  -Dpoint_msgs_DIR="$PWD/build/point_msgs_install/share/point_msgs/cmake"

cmake --build build --target sentry -j18
```
### 2. 开启导航通信
common:
  navigation:
    enable: true
    feedback_timeout_ms: 500.0
开启后，自瞄程序通过ROS收发，不会打开自己的串口；串口由导航程序负责。

### 3. 导航端适配消息
导航端也需要更新并重新编译同一份Message.msg，发布反馈时填入：
msg.big_yaw_in_world = received_data.big_yaw_in_world;

通信方向为：
下位机 → 导航串口程序 → /target_point → 自瞄
下位机 ← 导航串口程序 ← /auto_aim_plan ← 自瞄

### 4. 启动与检查
先启动导航串口程序，再在已加载上述环境的远程终端启动自瞄：
./bin/sentry sentry
另开远程终端检查：
source /opt/ros/humble/setup.bash
source /home/oy/dts_vision/build/point_msgs_install/share/point_msgs/local_setup.bash

ros2 topic hz /target_point
ros2 topic hz /auto_aim_plan
ros2 topic echo /auto_aim_plan

`/auto_aim_plan`预期约230Hz。未发现目标或反馈超过500ms未更新时，appear为0；导航端应据此停止执行有效瞄准/开火指令


/home/oy/dts_vision/script/auto_start/sentry.sh