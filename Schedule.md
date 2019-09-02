# Schedule for Master Thesis
This schedule provides orientation and is not finalized. 

## Fly in Simulation
1. [x] Generate drone race track in simulation.
2. [x] Generate trajectory through race track.
   * [ ] Prevent negative z with corridor constraints.
   * [ ] Nondimensionalization and scaling.
   * [ ] Optimal segment times.
3. [x] Launch Gazebo SITL simulation.
..* _Release v1.8.2 stable for SITL (my experience)._
4. [x] Load drone model with integrated camera.
..* _Launch SITL with Iris embedding front camera:_
```
roslaunch px4 posix_sitl.launch sdf:=/home/fm/src/Firmware/Tools/sitl_gazebo/models/iris_fpv_cam/iris_fpv_cam.sdf
```
5. [ ] Make the drone fly along agile trajectory.
6. [ ] Access camera data with ROS node.
7. [ ] Implement Neural network ROS node for dynamic trajectory planning.
8. [ ] Train and test the algorithm.
9. [ ] Expand algorithm onto other challenges (moving gates, different gates, ...).

* [ ] Fly drone in HITL simulation with RC transmitter. 

## Fly in Real World
1. [ ] Fly the drones via offboard control.
2. [ ] Mount and configure camera on drone.
3. [ ] Build a simple race track.
4. [ ] Train and test the algorithm after being approved in simulation.

## Further Ideas
1. [ ] Pose estimation from 2D images.