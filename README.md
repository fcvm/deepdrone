# re-implementation-deep-drone-racing
*With enhancements, extensions and transfers to other areas of application*


**Following**

E. Kaufmann, A. Loquercio, R. Ranftl, A. Dosovitskiy, V. Koltun, and D. Scaramuzza. Deep
drone racing: Learning agile flight in dynamic environments. arXiv preprint arXiv:1806.08548,
2018.

*Description*

They developed a supervised learning method which enabled a vision-based quadrocopter
to autonomously traverse through a drone-racing track with high agility. 
While the drone was exclusively trained in static environments, in test it was capable of
flying through moving race track gates, i. e. it handled navigating through **dynamic environments**.
Thereby, the drone did not require any explicit map of the environment, but
only localize itself relative to the next gate relying on camera images. All computation
run fully onboard.

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

## Questions and Problems
+ What if next gate is not seen in picture?
+ What if two gates are seen in picture?
+ What if perceptual aliasing appears? Buy better and more expensive hardware? Reduce computational cost by enhancing algorithm?

## Study more
+ Convolutional neural network (CNN)
+ State-of-the-art path-planning
+ Drones control system
+	SLAM




