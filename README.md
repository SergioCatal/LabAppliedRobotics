# Laboratory of Applied Robotics Project
## A C++ Localization and Path Planning program for a small Lego Mindstorm differential drive
Final Work of the Laboratory of Applied Robotics course held in the accademic year 2018/2019 by professor Luigi Palopoli and his teaching assistant Paolo Bevilacqua at the University of Trento.
This project has been developed by Simone Zamboni and Sergio Catalano: it took a very long time and a lot of sleepless nights to complete it, but in the end the result is worth the effort, as we hope you will see.

###### Challenge
There is a camera attached to the ceiling taking pictures like this:
![alt text](https://github.com/SZamboni/LabAppliedRobotics/blob/master/data/test_imgs/2018-12-19-175402.jpg)
The robot, which has a light-blue triangle on top of for localization, has to pass through all the green circles while avoiding the black walls and the red obstacles and then go to the blue rectangle, i.e. the exit. The camera is the only exteroceptive sensor that can be used for the task.

There are two types of mission which the robot must be able to complete: 
1. the robot has to pass on top of every green circle in the order dictated by the numbers inside the circles and then go to the exit in the least time possible
2. the robot has to reach the exit as fast as possible, but it obtains a bonus time score (e.g. -10 seconds) for every green circle it passes through. Therefore, in this kind of mission the robot has to compute which green circles are convenient to pick and wich are to ignore in order to reach the exit with the smallest time score.

###### Proposed solution
So the goal of the project was to implement a C++ program able to:
- localize the robot through the camera
- identify key objects in the environment through the camera
- compute an offline path either for mission 1 or mission 2
- follow the obtained path by combining wheel odometry and camera localization through a kalman filter

Our implementation reconstructs the map of the environment with OpenCV, then uses the Line Sweep algorithm to create a graph in the collision-free space, which is eventually used to compute a path composed entirely by Dubins Curves.

So we have this image from the camera:
![alt text](https://github.com/SZamboni/LabAppliedRobotics/blob/master/doc/readmeImg1.png)
We compute the logical map using OpenCV:
![alt text](https://github.com/SZamboni/LabAppliedRobotics/blob/master/doc/readmeImg2.png)
We inflate the obstacles so we can consider the robot as point:
![alt text](https://github.com/SZamboni/LabAppliedRobotics/blob/master/doc/readmeImg3.png)
And then we compute the best path using Dubins Curves:
![alt text](https://github.com/SZamboni/LabAppliedRobotics/blob/master/doc/readmeImg4.png)

###### Notes on project structure and implementation
In the folder "doc" you can find the final presentation (highly recommended if you want a more in detail overview of the project), the UML of all the classes and the documentation generated by doxygen.
In "data" there are some test data, in "include" there are all the .hpp files while in "src" there are all the .cpp files, in the "input" foldere there are the configuration file that the program reads every time it runs, and the makefile generates an executable called "lar".

