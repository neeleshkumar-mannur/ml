# Robot Motion Planning

## Project Description

Robot Motion Planning project is one of the capstone project suggestions in Udacity's Machine Learning Engineer Nanodegree Program. 
The project took inspiration from [Micromouse](https://en.wikipedia.org/wiki/Micromouse) competitions, wherein a robot mouse is tasked with plotting a path from a corner of the maze to its center.
The robot mouse may make multiple runs in a given maze. In the first run, the robot mouse tries to map out the maze to not only find the center, but also figure out the best paths to the center. In subsequent runs, the robot mouse attempts to reach the center in the fastest time possible, using what it has previously learned.

## Software Requirements
- Python 2.7
- NumPy

## Source codes

Codes for the project includes the following files:
- robot.py  
  This script establishes the robot class. 
- maze.py   
  This script contains functions for constructing the maze and for checking for walls upon robot movement or sensing.
- tester.py  
  This script will be run to test the robot’s ability to navigate mazes.
- showmaze.py  
  This script can be used to create a visual demonstration of what a maze looks like.
- test_maze_##.txt  
  These files provide three sample mazes upon which to test your robot. 

To execute the code and verify the model, execute the following command at the command line

-python tester.py test_maze_01.txt

To visualise a maze, execute the following command at the command line

-python showmaze.py test_maze_01.txt
