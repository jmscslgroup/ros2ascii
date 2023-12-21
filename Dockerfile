FROM osrf/ros:noetic-desktop-full


#ENV TERM linux
ENV TERM xterm-256color
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update 
RUN apt-get install -y curl nano iputils-ping net-tools git
RUN apt-get install -y libncurses5-dev libassimp-dev libbullet-dev pkg-config cmake build-essential libpng-dev

RUN cd home && git clone https://github.com/blegas78/curses-gfx 
RUN mkdir -p home/curses-gfx/build
RUN cd home/curses-gfx/build && cmake ..
RUN cd home/curses-gfx/build && make -j4 && make install
RUN ldconfig

RUN apt-get update
RUN apt-get install -y ros-noetic-tf2 ros-noetic-tf ros-noetic-geodesy python3-catkin-pkg


RUN mkdir -p home/catkin_ws/src
RUN cd home/catkin_ws/src/ && git clone https://github.com/jmscslgroup/ros2ascii
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd home/catkin_ws; catkin_make'


CMD ["/bin/bash"]