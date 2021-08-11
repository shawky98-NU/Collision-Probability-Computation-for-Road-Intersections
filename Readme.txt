In this paper, we aim to design a system in which vehicles
can transmit their information like speed, current position,
and turning direction to fixed infrastructure (RSU) via V2I
when they are near the intersection. When RSU receives the
information, it will predict the possible trajectory for both
vehicles, detect the possible collision point, and calculate TTC.
Then the collision probability can be calculated by using
TTC. We use the Simulation of Urban MObility (SUMO)
to simulate the vehicle’s motion at intersections and build
motion scenarios using MATLAB/Simulink to analyze the
data and compute the collision probability. All possible routes
are designed for each lane in the intersection by SUMO and
connect it with MATLAB by Traci4matlab library. This library
is useful to use all simulation outputs in MATLAB to calculate
the collision probability.