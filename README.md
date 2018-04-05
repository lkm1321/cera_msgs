# What is this? 

Message definition for CERA project

# How to build? 

Clone this repo to catkin_ws/src. Do catkin_make

There is an optional argument BUILD_MSG_TEST, which builds a simple test (msg_test) for publishing messages. To do this, 

```
catkin_make -DBUILD_MSG_TEST=ON 
source devel/setup.bash
roslaunch cera_msgs test.launch 
```

This should launch the test node + a rosbag recorder. 