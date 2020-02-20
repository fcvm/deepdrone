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
<!-- For Latex rendering: https://www.quicklatex.com/ -->
Angular velocities of frame B in frame W

![equation](https://quicklatex.com/cache3/e2/ql_eca73988599f3a2e91fed570c778aee2_l3.png)
&nbsp; &nbsp; **(1)**
<!-- \omega_{BW} = p \vec x_B + q \vec y_B + r \vec z_B -->

Control input
![equation](https://quicklatex.com/cache3/d5/ql_a7d5833b136f53c0f7ca70a9b4f975d5_l3.png)
&nbsp; &nbsp; **(2)**
<!-- \vec u = \begin{bmatrix}k_\text{F} & k_\text{F} & k_\text{F} & k_\text{F} \\0 & k_\text{F} L & 0 & - k_\text{F} L \\ k_\text{F} L & 0 & k_\text{F} L & 0 \\ k_\text{M} & - k_\text{M} & k_\text{M} & -k_\text{M} \\ \end{bmatrix} \begin{bmatrix}\omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \\ \end{bmatrix} -->

Newton's equation of motion
![equation](https://quicklatex.com/cache3/62/ql_e608cddf0a2a3b67530a6f00a111c562_l3.png)
&nbsp; &nbsp; **(3)**
<!-- m \ddot{\vec{r}} = -mg \vec z_W + u_1 \vec z_B -->

Euler equation
![equation](https://quicklatex.com/cache3/1e/ql_e0e0ed9dcf5d0d86df4c2b6a21bcf91e_l3.png)
&nbsp; &nbsp; **(4)**
<!-- \dot{\vec \omega}_{BW} = I^{-1} \left[ - \vec \omega_{BW} \times I \vec \omega_{BW} + \begin{bmatrix} u_2\\ u_3\\ u_4 \end{bmatrix} \right] -->

Trajectory: smooth curve in the space of flat outputs
![equation](https://quicklatex.com/cache3/35/ql_452526385cb285d2ef93f42d24340035_l3.png)
&nbsp; &nbsp; **(5)**
<!-- \sigma (t) : [t_0, t_m] \rightarrow \mathbb{R}^3 \times SO(2),\ \sigma = \begin{bmatrix} x,y,z,\psi \end{bmatrix}^T -->




## References
1. E. Kaufmann, A. Loquercio, R. Ranftl, A. Dosovitskiy, V. Koltun, and D. Scaramuzza. Deep drone racing: Learning agile flight in dynamic environments. arXiv preprint arXiv:1806.08548, 2018.
