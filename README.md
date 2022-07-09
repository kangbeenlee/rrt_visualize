# rrt_visualize
RRT visualization with Rviz

<br/>

# Clone this repository in your working directory
In home directory

```
mkdir -p rrt_ws/src
git clone https://github.com/kangbeenlee/rrt_visualize.git
cd ..
catkin_make
```

<br/>

# Implementation Rviz
In terminal
```
roscore
```

In new terminal
```
source devel/setup.bash
rosrun rrt_visualize RRT
```

In new terminal
```
rviz
```

In rviz
change cell size 100
Add Marker
