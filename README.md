# Clone this repository in your working directory
### In home directory

```
mkdir -p rrt_ws/src
git clone https://github.com/kangbeenlee/rrt_visualize.git
cd ..
catkin_make
```

<br/>

# Implementation Rviz
### 1. In terminal
```
roscore
```

### 2. In new terminal
```
source devel/setup.bash
rosrun rrt_visualize RRT
```

### 3. In new terminal
```
rviz
```

### 4. In rviz
* Change cell size 100
* Add Marker

![rviz_setting](https://user-images.githubusercontent.com/81845189/178102797-dc9a4e80-f98c-4d3e-ae8b-23b6478a40d1.png)

<br/>

* now you can see the rrt visualization

![rrt_rviz](https://user-images.githubusercontent.com/81845189/178102796-f538a629-14b4-441e-943d-0b05fba87179.png)
