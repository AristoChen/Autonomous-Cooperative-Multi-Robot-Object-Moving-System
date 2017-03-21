# Autonomous Cooperative Multi-Robot Object-Moving System(ACMOS)

ACMOS is designed to complete object-moving tasks assigned by humans, and without any control after tasks assigned. ACMOS will send appropriate number of robots to complete the task according to the size and weight of the object.

This System contains ControlSystem and RobotSystem, and the following is the Architecture of the system

![SystemArchitecture](http://i.imgur.com/LpifSnsm.jpg)

The ControlSystem should be located at the top of the space as you can see in the following picture

![SystemArchitecture(Physical)](http://i.imgur.com/0Z3DCnxm.jpg)

Each color refers to different role
 - Red : ControlSystem
 - Orange : RobotSystem
 - Blue : Object
