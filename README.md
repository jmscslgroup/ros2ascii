# ros2ascii
Terminal software graphics visualizer for ROS using ASCII art

Code is in the form of C++ ROS nodes.


This is based on the curses-gfx library here: [https://github.com/blegas78/curses-gfx](https://github.com/blegas78/curses-gfx)

Also see ROS nodes from the can_to_ros project that this is built upon: [https://github.com/jmscslgroup/can_to_ros](https://github.com/jmscslgroup/can_to_ros)

ROS nodes in this package:

### radar2ascii

On a RAV4 running libpanda and can_to_ros, subscribes to topics /tracka0, /tracka1, ... /tracka15 and draws blue 3D boxes.  Also draws a cyan representation of the car.

![Chaos](https://github.com/jmscslgroup/ros2ascii/blob/main/docs/images/radar2ascii2.png?raw=true)

Press the spacebar to toggle auto camera rotate, then:
WASD: move camera
Arrow keys: Pan/Tilt
T/R: zoom in/out
G: toggle grid
Escape: quit





