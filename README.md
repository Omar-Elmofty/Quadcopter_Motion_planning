# Motion Planning for Quadcopter Flying though narrow gaps

AER1516 - Motion planning for robotics - Final project, link for project
report and demonstration video:[Motion Planning for Quadcopter Flying though narrow gaps](https://drive.google.com/open?id=1OiX6Z48XeYNF04TpmMopRAOPYLLoRYya)

## Requirements 
Python convex optimization library [CVXOPT](https://cvxopt.org/)

## Implementation Example
To run the motion planner use the following command

`$ python main.py --max_iter 1000 --optim_type 'constrained' --steer_order 1`

To change obstacle shapes and locations edit environment.py

## Acknowledgments 

Course material from UTIAS AER1516 - Motion Planning for Robotics
    
Course material and drone simulator from UTIAS AER1217 - Autonomous UAS course 

Full references list can be found in the final [report](https://drive.google.com/open?id=1OiX6Z48XeYNF04TpmMopRAOPYLLoRYya)

Finally, special thanks for the authors of the following paper as it was influential for the work done in the project

Richter, Charles, Adam Bry, and Nicholas Roy. “Polynomial
Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor
Environments.” Robotics Research. Ed. Masayuki Inaba and Peter
Corke. Vol. 114. Cham: Springer International Publishing, 2016. 649




