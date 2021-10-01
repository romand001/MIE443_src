# MIE443 ROS Projects

## Contest 1

A simulated TurtleBot autonomously navigates and maps an environment.
- Occupation Grid processing was done to aid in navigation.
  - Morphological operations to make sure the robot does not collide with obstacles.
  - Convolution smoothing (values used in A*) so that the robot prefers to keep distance from obstacles.
- BFS-based frontier exploration algorithm detects the closest frontier border at each time step.
- A* algorithm plans a path from the robot to the center of the frontier border, using processed Occupancy Grid values as additional cost in order to prioritize open spaces.
- PID controller guides the robot along the path.
- Additional checks for emergency situations, and subsequent recovery behaviour were also implemented.

![Contest 1 image](https://i.ibb.co/4f7GvZ6/Contest-1-example-small.jpg)

## Contest 2

A simulated Turtlebot must navigate to and classify images in an environment within an allotted time.
- Brute-force algorithm finds the optimal path for navigating to all images.
- Custom algorithm navigates to location where the image is within FOV at an appropriate angle and distance.
- SURF feature detector extracts features from a cropped view and compares with the image database.
- Image classification algorithm select which database image it is seeing.

![Contest 2 optimal path](https://i.ibb.co/Ydrz6py/Contest-2-example-1-small.png)
![Contest 2 SURF detection](https://i.ibb.co/1Mh346R/Contest-2-example-2-small.png)

## Contest 3

A simulated TurtleBot must explore an unknown environment and find 7 victims, classify their emotions, and interact approproately.
- Open-source frontier exploration algorithm was modified for improved performance and reliability.
- CNN was trained to classify 10 emotions.
- K-Folds Cross Validation used to select the best CNN model.
- Motion, sound, and visual interaction is performed for each victim.

![Contest 3 image](https://i.ibb.co/7V8D8Gc/CNN-Model.png)
