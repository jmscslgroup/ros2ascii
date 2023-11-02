# ros2ascii
Terminal software graphics visualizer for ROS using ASCII art

Code is in the form of C++ ROS nodes.  Only tested on Raspbian with ROS Noetic.


This is based on the curses-gfx library here: [https://github.com/blegas78/curses-gfx](https://github.com/blegas78/curses-gfx)

Also see ROS nodes from the can_to_ros project that this is built upon: [https://github.com/jmscslgroup/can_to_ros](https://github.com/jmscslgroup/can_to_ros)

ROS nodes in this package:

### radar2ascii

On a RAV4 running libpanda and can_to_ros, subscribes to topics /tracka0, /tracka1, ... /tracka15 and draws blue 3D boxes for slow opbejct and red buxes for hfaster objects. A yellow box represents the boundaries of the car (2020 Toyota RAV4).  Uses OpenStreetMaps to show current location within a map, along with the GPS module 3D covariance bubble.  See video over a WiFi SSH terminal:

[![ros2ascii](https://img.youtube.com/vi/yZGyJVOFqZc/maxresdefault.jpg)](https://youtu.be/yZGyJVOFqZc)

Press the spacebar to toggle auto camera rotate, then:

WASD: move camera

Arrow keys: Pan/Tilt

T/R: zoom in/out

G: toggle grid

Escape: quit





