# FTG Node - ROS2 Gap Follower for F1Tenth in Autodrive Sim

## Disclaimer and Terms of Use
I hold no guarantee or liability whatsoever for any use of the code within the simulator. In addition, this code has not been tested outside the simulator, therefore any adaptation and use in real life will come at your own risk.

You may use this code freely for non-commercial use given an appropriate and prominent reference to this repository in a visible part of your documentation.

## Introduction

This is a Follow-The-Gap node for simple, reactive, non-map-based navigation based of a Lidar-visible track, subscribing to a 2D ROS2 LaserScan, doing neighbor averaging (to reduce the effect of any noise, though not present in the Autodrive simulator), determining the largest gap and its midpoint, and determining the angle accordingly. Also, the throttle is interpolated based on the angle and how close the car is to a possible obstacle.

This code was made to specifically run on ROS2, and interact with the Autodrive F1tenth simulator. The code accepts a Laserscan based on the topic of the simulator, and publishes a Float32 between [0,1] to the throttle actuator and the steering actuator.  If using outside the simulator, you may need to adapt the code to accept an Ackermann message.

The current benchmarking performance is 13 crashes over 272 laps in one go in 51min and 15sec, with average time of 11.29 seconds and a best lap of 10.3 seconds. Approximately, you will see about 1 crash every 20 laps. In order to reduce the crashes, you may reduce the decay value shown in Line 99 from 0.9 to a lower value though it may come at an expense of lap time.

## Running (for Autodrive container)

To quickly run the code, you can copy the file into the Docker container for the Autodrive devkit, and directly run `python3 ftg_src.py`. Alternatively it is recommended to integrate the node within your packages.
Please ensure the simulator is open, and Autodrive bridge is connected accordingly as the algorithm will not run if it is not able to subscribe to the simulator topics.

## Questions

If any issues arise while running please contact me via raising an issue on this repo.

Alternatively you may contact me at info@elyasshadi.com for further inquiries.




