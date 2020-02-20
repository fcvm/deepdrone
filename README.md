# Autonomous, vision-based navigation of MAVs with powers of recall
A navigation system for micro aerial vehicles (MAVs) with a forward facing onboard camera which should empower MAVs to autonomously traverse through a drone-racing track with high agility and high robustness. The navigation system is mainly based on the work of Kaufmann et al. (reference 1) who achieved: 
- high agility
- coping with dynamic environments
- onboard real-time implementation.

During testing, MAVs do not require a map of the environment because they localize themselves relatively to the next racing gate appearing on the current image from the camera. In addition to their work, this navigation system should not only input the current image but also elapsed images in order to increase robustness against feature loss, i.e., no racing gate is located in the frame of view (FOV) of the camera. This should be realised by forwarding the output of a convolutional neural network (CNN) to a long-short-term-memory network (LSTM). To my knowledge, the use of memory in autonomous navigation of MAVs is widely unexplored.


## Setup
```git
git clone <<Firmware>>
git tag -l
git checkout v1.8.2 #This release was the latest stable one in my experience
git submodule sync --recursive
git submodule update --init --recursive
```


## Run Simulation
Launch Gazebo simulation with standard quadcopter (model: Iris). In a terminal:
```bash
roslaunch px4 mavros_posix_sitl.launch
```
Horizontal takeoff and landing. In the PX4 system console (same terminal):
```bash
commander takeoff
#and
commander land
```
Track pre-defined waypoints: In a new terminal:
```bash
rosrun offb offb_node
#or
rosrun drone_racetrack offb_traj.py
```
---

Launch Gazebo simulation with quadcopter (model: Iris) with front camera. In a terminal:
```bash
roslaunch px4 posix_sitl.launch sdf:=/home/fm/src/Firmware/Tools/sitl_gazebo/models/iris_fpv_cam/iris_fpv_cam.sdf
```
---

Launch Gazebo simulation with drone racetrack generator. In a terminal:
```
roslaunch drone_racetrack drt.launch
```

---

Launch Gazebo simulation with quadcopter (model: Iris) with controller that tracks circular trajectory. In a terminal:
```
roslaunch geometric_controller sitl_trajectory_track_circle.launch
```



## Write Thesis
Launch JabRef. In a terminal:
```
java -jar Desktop/Programs\ and\ Services/JabRef-4.3.1.jar
```
---


## Minimum snap trajectory generation and control for quadrotors
### Equations
Angular velocities of frame B in frame W
<img src="https://latex.codecogs.com/gif.latex? \omega_{BW} = p \vec x_B + q \vec y_B + r \vec z_B" /> 


\[
SE = \frac{\sigma}{\sqrt{n}}
\]

![equation](https://quicklatex.com/cache3/e2/ql_eca73988599f3a2e91fed570c778aee2_l3.png)



## References
1. E. Kaufmann, A. Loquercio, R. Ranftl, A. Dosovitskiy, V. Koltun, and D. Scaramuzza. Deep drone racing: Learning agile flight in dynamic environments. arXiv preprint arXiv:1806.08548, 2018.
