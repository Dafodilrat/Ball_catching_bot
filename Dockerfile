FROM ghcr.io/sloretz/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    libgflags-dev ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher \
    libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev udev \
    ros-humble-diagnostic-updater ros-humble-geographic-msgs \
    build-essential cmake libgeographic-dev nlohmann-json3-dev

RUN git clone https://github.com/libuvc/libuvc.git \
    && cd libuvc && mkdir build && cd build \
    && cmake .. && make -j4 && make install && ldconfig

COPY ros2_ws /ros2_ws
WORKDIR /ros2_ws

RUN . /opt/ros/humble/setup.sh && colcon build --executor sequential --packages-ignore robot_localization astra_camera
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select astra_camera robot_localization

WORKDIR /ros2_ws/src
RUN cd ros2_astra_camera/astra_camera/scripts && chmod +x ./install.sh

WORKDIR /ros2_ws
CMD bash -c ". /opt/ros/humble/setup.sh && . ./install/setup.sh && ros2 launch pkg_motor_driver start_base.launch.py"