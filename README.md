# rrt_visualize
RRT visualization with Rviz

# Clone the rrt_visualize repository in your working directory
In home directory

'''c
mkdir -p rrt_ws/src
git clone https://github.com/kangbeenlee/rrt_visualize.git
cd ..
catkin_make
'''

# Implementation Rviz
In terminal
'''c
roscore
'''

In new terminal
'''c
source devel/setup.bash
rosrun rrt_visualize RRT
'''

In new terminal
'''c
rviz
'''

In rviz
change cell size 100
Add Marker
