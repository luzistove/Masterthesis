# Masterthesis
In this repository are the code and data of my master thesis with topic "Development and Evaluation of a learned biped Walking Trajectory based on 3D points on NAO Robotic System". First, several existing biped walking models from three different papers are simulated and compared. They are:
1. Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point 
https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1241826
2. A Closed-loop 3D-LIPM Gait for the RoboCup Standard Platform League Humanoid
https://www.b-human.de/downloads/Humanoids-Graf-Roefer-10.pdf
3. rUNSWift Walk2014 Report Robocup Standard Platform League
http://cgi.cse.unsw.edu.au/~robocup/2014ChampionTeamPaperReports/20140930-Bernhard.Hengst-Walk2014Report.pdf

After comparison, the best model is chosen out to implement walking motion with imitation learning. The walking motion of human is recorded by a Kinect depth camera, with which the 3D coordinates of body joints can be detected and stored as time sequences. Movement primitives such as forward and sideward walking steps are extracted from the recorded sequences. 
Robot executes the movement primitives with imitation learning using a technique called "Dynamic Movement Primitives" from Stefan Schaal's lab. Relativ theories can be found in:
1. Dynamic Movement Primitives–A Framework for Motor Control in Humans and Humanoid Robotics
https://s3.amazonaws.com/academia.edu.documents/30889910/10.1.1.142.3886.pdf?AWSAccessKeyId=AKIAIWOWYYGZ2Y53UL3A&Expires=1553349729&Signature=K%2FY%2BPXtJdVAmWALMbi5ocsIuhnI%3D&response-content-disposition=inline%3B%20filename%3DDynamic_movement_primitives-a_framework.pdf
2. Dynamical Movement Primitives: Learning Attractor Models forMotor Behaviors
http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.646.9754&rep=rep1&type=pdf

When the primitives are not good enough (slow, not moving straight, too large CoM swing etc.), they can be improved by using reinforcement learning. There is a reinforcement technique called "PoWER", especially for movement primitives:
1. Learning Motor Primitives for Robotics
https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5152577
2. Policy Search for Motor Primitives in Robotics
https://papers.nips.cc/paper/3545-policy-search-for-motor-primitives-in-robotics.pdf
