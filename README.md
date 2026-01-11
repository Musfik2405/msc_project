 Gathering on a Circle with Limited Visibility by Anonymous Oblivious Robots
1. Introduction
This project studies the gathering problem in distributed robotics. The gathering problem means
making all robots come to a single point without direct communication. The robots used in this
project are very simple and have limited hardware capabilities. They move only on a circular
path and depend completely on their sensors to observe other robots.
The study helps in understanding how minimal sensing and computation can still allow
coordination in swarm robotics.
2. Robot Model and Sensors
Each robot is equipped with the following sensors:
• Vision Sensor (Angular Visibility Sensor):
This sensor allows a robot to detect other robots within a fixed angular range on the
circle. The robot cannot see robots outside this range.
• Position Sensor (Relative Position Detection):
The robot can determine the relative positions of visible robots along the circle but does
not know exact distances in a global coordinate system.
• Orientation Sensor (Compass / Chirality Sensor):
This sensor helps the robot distinguish clockwise from counter-clockwise directions.
The robots do not have:
• Memory sensors (no storage of past states)
• Communication devices
• Identity sensors
3. Problem Description
Robots are placed at different points on the circumference of a circle. Each robot repeatedly
performs a Look–Compute–Move cycle:
1. Look: The vision sensor detects nearby robots.
2. Compute: The robot decides a movement direction based only on current observations.
3. Move: The robot moves along the circle.
The main goal is for all robots to gather at one point on the circle.
4. Methodology
The project uses mathematical and logical analysis to study robot behavior under a semisynchronous model, where some robots may move while others remain still. Different visibility
limits are analyzed to understand how sensor range affects coordination.
Algorithms are designed when gathering is possible, and formal proofs are used to show
impossibility when it is not.
5. Main Results
• When the vision sensor range is up to π radians, robots can observe enough of the circle
to coordinate their movements and successfully gather at a single point.
• When the visibility range is π/2 radians or less, the robots lack sufficient information,
making gathering impossible even if the robots are initially well distributed.
This shows that the sensing capability of robots directly determines whether gathering can be
achieved.
6. Conclusion
The project demonstrates that simple robots with limited sensors can achieve gathering only if
their vision sensors provide sufficient coverage of the environment. When visibility is too
restricted, coordination becomes impossible due to lack of information. These findings are
important for designing low-cost swarm robotic systems with minimal sensing hardware.
Prepared by:
Md tanjimul Islam
A.B.M Rafiul Hasan
MD. Mushfikur Rahman
