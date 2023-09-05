# README

## Рекоммендации к установке

- Ubuntu 20.04
- ROS Noetic

## Установка

Установить зависимости:
```shell
# Предполагается, что среда ROS подготовлена (source /opt/ros/noetic/setup.bash)
sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
```

Установить libuvc:
```shell
cd ~
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir ./build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
cd ~
```

Склонируйте и соберите воркспэйс с пакетами, необходимыми для лидара и камеры:
```shell
git clone https://github.com/Yyote/aerobot_ws.git --recursive
cd aerobot_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
echo "source ~/aerobot_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Запуск

Для запуска лидара:
```shell
roslaunch turn_on_wheeltec_robot wheeltec_lidar.launch 
```

Для запуска камеры:
```shell
roslaunch turn_on_wheeltec_robot wheeltec_camera.launch 
```
