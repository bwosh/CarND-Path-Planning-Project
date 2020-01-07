# CarND-Path-Planning-Project
This project is a part of:  
 [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Goal
The goal of the project is to navigate car car in highway traffic to:
- keep reasonable speed (around 50 MPH)
- not violate speed limit of 50 MPH
- plan path not to accelerate/decelerate too much (max 10 m/s^2)
- plan path not to jerk too moch (max 10 m/s^3)
- not to have colissions with other cars
- to change lanes whe it's reasonable
   

### Simulator.
This code is part of solution together with  [Term3 Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  


### Building and running the code.

To build the code that communicate with the simulator:
- Make build directory
- enter build directory 
- build code by 'cmake .. && make'
- run path_planner


# Description of the solution

## Input data
The data from telementry contained:
- Car's x position on the map
- Car's y position on the map
- Car's s position in Frenet coordinates
- Car's d position in Frenet coordinates
- Car't yaw rate
- Car's speed magnitude


## Path creation 
Two first points are created either by:
- taking last 2 previous points from last path  

or by :
- adding virtual previous point when path planning begins based on yaw of the car to produce tangent path

## Sensor fusion data
Sensor fusion data contains information about all other vehicles around the ego car. It contains: id, position, vx and vy speeds, s & d in Frenet coordinate system.

The data is used to calculate all lanes clearance data. Yhis measures for each lane how far the next car is ahead of car and back of the car. Slowing down is applied basing on velocity magnitude of car ahead if it's in proper distance.

Since Frenet coordination system is circural it changes value at the end of scale. All coordinations of cars at the edge are normalized.  

## Lane switching
To address lane switching simple algorithm was used that:
- checks the distance from car ahead
- checks if there is any adjacent lane that is worth using (there is at least X more meters to use)
- checks limits of clearance (looking front and looking back separately)

## Coordinates transformation
To plan next points coordinate system is transformed so the car is placed at the x axis of two-dimensional space.

Just before adding the final points to the list the operation is reversed to use real map locations.

## New points creation

The horizon of path generated points is set 30 meters ahead of the car. Three more points are planned in 30m, 60m, 90m ahead of the track basing on the track waypoints. 

All 5 points are used to generate curve that is smooth enough to be used. Spline library is used to generate points.

Desired speed is set to maximum speed or speed of the car ahead. Basing on speed, velocity & acceleration calculations the maximum range of distance that does not break acceleration/jerk is calculated, both to x and y axis. If the generated smooth line exceeds the proper range the fix is applied so the acceleration and jerk stay in safe space.

# Discussion

Used lane switching conditions is really safe. It needs lots of clearance to change the lane. More research and work could be put to incorporate all vehicles speeds and future positions to make more drastic decisions.

What hasn't beed addressed here is changing lanes by more that one lane. In theory it may happen that lane on the other side is much better that the second one on the other side yet there car in between that could be skipped by lowering the speed. Yet that planning would also need to analise all other positions of cars in the back.


