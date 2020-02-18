# Autonomous, vision-based navigation of MAVs with powers of recall
A navigation system for micro aerial vehicles (MAVs) with a forward facing onboard camera which should empower MAVs to autonomously traverse through a drone-racing track with high agility and high robustness. The navigation system is mainly based on the work of Kaufmann et al. (reference 1) who achieved: 
- high agility
- coping with dynamic environments
- onboard real-time implementation. 
During testing, MAVs do not require a map of the environment because they localize themselves relatively to the next racing gate appearing on the current image from the camera. In addition to their work, this navigation system should not only input the current image but also elapsed images in order to increase robustness against feature loss, i.e., no racing gate is located in the frame of view (FOV) of the camera. This should be realised by forwarding the output of a convolutional neural network (CNN) to a long-short-term-memory network (LSTM). To my knowledge, the use of memory in autonomous navigation of MAVs is widely unexplored.


## References

1. E. Kaufmann, A. Loquercio, R. Ranftl, A. Dosovitskiy, V. Koltun, and D. Scaramuzza. Deep drone racing: Learning agile flight in dynamic environments. arXiv preprint arXiv:1806.08548, 2018.



## Ideas for Extensions

+ Cope with high speed. Handle motion blur by integrating in training. Develop a general way.
+ The end-to-end learning approach in the paper performed slowly and badly. Enhance this appraoch. Integrate penalty for slowness.
+ Hide and Seek.
+ Reversed ionic wind for convert wind into eletrical energy.
+ Design a training procedure in dynamic environments. Compare performance with training in static environments which also generalize to dynamic environments.
+ Visual-based safe drone convoy navigation. Only the first drone is required to comprehensively autonomously navigate. Leader drones.  
+ Visual-based autonomous anti-collide algorithm for two drones approaching each other. (+Which drone passes gate first).
+ Cope with severe occlusions.
+ Cope with highly dynamic environments.
+ Deal with challenging lighting systems. Mount Headlights with color-alternating light.
+ Use this approach to correct drift of visual odometry-based approaches.
+ *While our current set of experiments was conducted in the context of drone racing, we believe that
the presented approach could have broader implications for building robust robot navigation systems
that need to be able to act in a highly dynamic world. Methods based on geometric mapping,
localization and planning have inherent limitations in this setting. Hybrid systems that incorporate
machine learning, like the one presented in this paper, can offer a compelling solution to this task,
given the possibility to benefit from near-optimal solutions to different subproblems.
Scaling such hybrid approaches to more general environments and tasks is an exciting avenue for
future work that poses several challenges. First, while the ability of our system to navigate through
moving or partially occluded gates is promising, performance will degrade if the appearance of the
environment changes substantially beyond what was observed during training. Second, in order
to train the perception system, our current approach requires a significant amount of data in the
application environment. This might be acceptable in some scenarios, but not practical when fast
adaptation to previously unseen environments is needed. This could be addressed with techniques
such as few-shot learning. Third, in the cases where trajectory optimization cannot provide a policy
to be imitated, for instance in the presence of extremely tight turns, the learner is also likely to fail.
This issue could be alleviated by integrating learning deeper into the control system.*

## Questions and Problems
+ What if next gate is not seen in picture?
+ What if two gates are seen in picture?
+ What if perceptual aliasing appears? Buy better and more expensive hardware? Reduce computational cost by enhancing algorithm?

## Study more
+ Convolutional neural network (CNN)
+ State-of-the-art path-planning
+ Drones control system
+	SLAM

## Setup
git clone <<Firmware>>
git tag -l
git checkout v1.8.2 #My current choice
git submodule sync --recursive
git submodule update --init --recursive

## Run Simulation


```

```


Launch Gazebo simulation with standard quadcopter (model: Iris). In a terminal:
```bash
roslaunch px4 mavros_posix_sitl.launch
```
Horizontal takeoff and landing. In the PX4 system console (same terminal):
```bash
commander takeoff
commander land
```
Track pre-defined waypoints: In a new terminal:
```bash
rosrun offb offb_node
```
---

Launch Gazebo simulation with quadcopter (model: Iris) with front camera. In a terminal:
```bash
roslaunch px4 posix_sitl.launch sdf:=/home/fm/src/Firmware/Tools/sitl_gazebo/models/iris_fpv_cam/iris_fpv_cam.sdf
```


