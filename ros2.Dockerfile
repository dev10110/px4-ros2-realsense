FROM chenrc98/ros2-px4-pi:version1.1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends vim tmux

RUN apt-get install -y --no-install-recommends \
  ros-galactic-realsense2-camera

RUN apt-get install -y --no-install-recommends \
  libboost-all-dev

RUN apt-get install -y --no-install-recommends \
  libpcl-dev 

RUN rm -rf /var/lib/apt/lists


## install octomap
WORKDIR "/root/"
RUN git clone https://github.com/OctoMap/octomap.git
WORKDIR "/root/octomap/build"
RUN cmake .. && make -j
# set an env variable on where octomap is
ENV octomap_DIR /root/octomap/lib/cmake/octomap/

# px4 stufff
RUN echo "source /opt/ros/galactic/setup.bash" >> /root/.bashrc
RUN echo "alias ss='source /root/px4_ros_com_ros2/install/setup.bash'" >> /root/.bashrc
RUN echo "alias bridge='micrortps_agent -d /dev/ttyAMA1 -b 921600 -n drone1'" >> /root/.bashrc

WORKDIR "/root/px4_ros_com_ros2"
