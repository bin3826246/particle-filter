# Overview
This repository achieves a 2-D particle filter of a self-driving vehicle in C++. The particle filter will be given a map of landmark and an initial localization. The particle filter will get noisy control and observation data. And then the program will calculate and output best particle location.

Finally, draw particle trajectory diagrams using Python scripts.
# Running
run `src/main.cpp` - calculate the best particle location

run `draw.py` - draw the particle trajectory diagrams

# Input Data
```
control_data.txt : 
v  yaw

where:
(v  yaw) - the velocity and yaw rate of the sensor.

gt_data.txt : 
x y theta

where:
(x y theta) - the ground truth data of XY coordinates and yaw of the vehicle.

map_data.txt : 
landmark_x landmark_y id

where:
(landmark_x landmark_y id) - the data of XY coordinates and id of landmarks.

observation_*.txt : 
local_x, local_y

where:
(local_x, local_y) - observation data of sensor at each time step.
```
# Result
![pipeline](./pic/Figure_1.png)
# References
https://zhuanlan.zhihu.com/p/107223012
