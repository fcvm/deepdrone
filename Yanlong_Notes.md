# Notes of Meeting with Yanlong
Information on launching HITL simulation and fly the drone in real world.

## HITL Simulation
1. Wifi antenna
2. Power Cable (red)
3. QGroundControl (connected if red drone appears on screen)
⋅⋅* Flight mode: hold
⋅⋅* Arming
4. POSIX SITL Gazebo

## Emergency Action
Disarm drone during flight to initiate controlled crash.

## Battery Charging
For Charging contact Jiayu.


## Fly in Real World
Before mavros do check with QGC.
QGC and mavros use same address.
Check if GPS is stable (rostopic echo mavros/local_position/pose).
Drone has a safety switch.


## Communication Scheme

LAPTOP --------------- WIFI ROUTER
   |                          |
   |                          |
   |                          |
 <QGC> (Port?)               <Script, Remote Control>
   |                          |
   |                          |
   |                          |
  PX4 <--------------------> TX2 (Ubuntu 16)

