# MuJoCo Simulator for Belt-Augmented Compliant Hand (Yilin Cai, Shenli Yuan 2023)
### This repository tracks the research progress of an attempt to computationally simulate the BACH manipulator.
#### Student: Louis Abbott
#### Research Mentor: Podshara Chanrungmaneekul
#### Lab: RobotΠ Lab, directed by Dr. Kaiyu Hang

## Overview
A simplified version of the BACH's kinematic tree was imported directly from OnShape, via OnShape-to-robot (https://onshape-to-robot.readthedocs.io/en/latest/index.html). It's worth noting for future use that the mates must be made in the outermost level of the assembly tree in order for this program to work. The resulting URDF was converted to MJCF, an XML langauge native to MuJoCo.
