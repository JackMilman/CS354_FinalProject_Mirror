YouTube Demo: https://youtu.be/VBasxtw7xyE?si=soy-95dyyPrVllpI
Current State of Code:
Currently, the robot randomly selects a goal position and then navigates to that position. We decide on a number of 
positions we want the robot to anvigate to, and it then repeats this process for the specified number of goal positions. 
Once it reaches the final goal position, it navigates back to the starting position, matching the original coordinate and angle. 
At this point in the project, we are working on using the ArUco package so the robot is able to identify victims and keep track of their positions.
Although our victim identification is not working at this point, we are currently trying to figure out how to make it work.
To run our code, you need to clone all of the packages into src, then cd into dev_ws and do colcon build --symlink-install, then source install/setup.bash.

Checkpoint 2 Goal:
For checkpoint 2, our plan is to first get our robot to successfully identify the victims using the augmented reality markers. 
Once our robot is able to keep track of the victims locations, we then need to implement a more efficient navigation algorithm.
Right now, our robot navigates by randomly selecting positions to check, which is not ideal when our victims. In order to locate them
as quickly as possible, we are currently looking into which algorithms will be most efficient.
