# Mobile_robot_controllers
The package includes controllers (mainly nonlinear types) for the mobile robot (omni type). Both kinematic and dynamic controllers are implemented. The overall system is based on ROS.


- Try to build robot model can perform with each torque wheel

- pid controller: not perform well because robot model is nonlinear
- kinematic controllers: best performance but with vx vy w. Need to test with wheel velocity as input
- dynamic controllers: good performance but with more tracking error than kinematic controllers. Input is forces along three axes
