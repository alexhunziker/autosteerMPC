# Autonomous Steering of an Electric Bicycle Based on Sensor Fusion Using Model Predictive Control

Master Thesis of Alex Hunziker, Institute of Computer Science Goethe University Frankfurt (December 2019)

The thesis can be found here: [Thesis Full Text](https://github.com/alexhunziker/autosteerMPC/blob/master/ThesisAutonomousBicycleMPC.pdf)

## Abstract

In this thesis a control and steering module for an autonomous bicycle was developed.
Based on sensor fusion and model predictive control, the module is able to trace routes
autonomously.

The system is developed to run on a Raspberry Pi. An ultrasonic sensor and a 2D Lidar
sensor are used for distance measurements. The vehicle’s position is determined by using
GPS signals. Additionally, a camera is used to capture pictures for the roadside detection.
In order to recognize the road and the position of the vehicle on it, computer vision techniques are
used. The captured images are denoised, Canny edge detection is performed and a perspective transformation is applied. Thereafter a sliding window algorithm selects
the edges belonging to the roadside and a second order polynomial is fitted to the selected
data. Based on this, the road curvature and the lateral position of the vehicle on the
road are calculated. The implemented software is thus able to detect
straight and curved roads as well as the vehicle’s lateral offset. 

A route planning module was implemented to navigate the vehicle from the start to the
destination coordinates. This is done by creating an abstract graph of the roads and using
Dijkstra’s algorithm to determine the shortest path.

Four MPC controllers were implemented to control the movements of the vehicle. They
are based on state space equations derived from the linear single-track vehicle model. This
relatively straightforward model makes it possible to predict the vehicle behavior and is
efficient to compute. Each controller was built with different parameters for different
vehicle speeds to account for the non-linearity of the system. The controllers simulate
the future states of the system at each timeslot and select appropriate control signals for
steering, throttle and brakes. Using multiple models and controllers makes the system
more resilient, as it is able to react properly in situations where not all data is available.

In this thesis, all the components of the steering and control module were individually
validated. It was established that the each individual component works as expected and certain constraints and accuracy limits were identified. Finally, the closed loop capabilities
of the system were assessed using a test vehicle. Despite some limitations imposed by this
setup, it was shown that the control module is indeed capable of autonomously navigating
a vehicle and avoiding collisions.
