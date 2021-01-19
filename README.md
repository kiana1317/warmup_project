# Warmup_project
## Writeup Topics

* For each robot behavior, describe the problem and your approach at a high-level. Include any relevant diagrams or pictures that help to explain your approach.
* Describe the structure of your code. For the functions you wrote, describe what each of them does.
* While recording your robot's behavior in a rosbag conducting each type of behavior, also record a gif of the robot visually. Include this gif in your writeup and use it for analysis if needed. For instructions on how to make a gif recording, look at Gazebo simulator.
* Describe the challenges you faced and how you overcame them.
* If you had more time, how would you improve your robot behaviors?
* What are your key takeaways from this project that would help you/others in future robot programming assignments? For each takeaway, provide a few sentences of elaboration.

## Driving in a Square
### Description
The goal of this robot behavior was to get the robot to drive in a square pattern. To achieve this, I used both timing and the angular velocity formula to gauge at what point while turning the robot would have reached a 90-degree angle. However, the one aspect that complicated this behavior was that the robot does not come to a hard stop nor automatically reaches any designated velocity. Therefore, to resolve this discrepancy, the robot's velocity had to be adjusted slightly from the results of the [angular velocity formula](https://www.omnicalculator.com/physics/angular-velocity) to offset those inaccuracies. In the end, the robot moves forward a few seconds, rotates 90 degrees, and then repeats, completing a nearing exact square.
### Code Structure
I structured my code following the object-oriented layout outlined during class. As such, I initialized the class by calling the publisher node "/cmd_vel" to gain access to the robot's linear and angular velocity: the linear velocity for moving the robot forward and the angular velocity for shifting directions. Moreover, I added in a 1 second pause to ensure the publisher is fully connected before publishing any data to it. <br /> 
For the run function, I created a while loop that instructs the robot to move forward for a moment and then turn counterclockwise by 90 degrees. To send the robot forward, a linear x rate of 0.1 is published to the publisher. After a 4 second interval, the linear velocity is halted, and the angular velocity is changed to 0.3980 to facilitate a 90 degree turn over the course of 4 seconds. This loop is repeated to send the robot on a square path. As mentioned earlier, the variable offset is the value added to the calculated angular velocity to compensate for the discrepancies in the stopping and accelerating mechanisms in the robot.
### Demo 
![Alt Text](https://raw.githubusercontent.com/kiana1317/Warmup-Intro-to-Robotics/main/gifs/drivesquare.gif?token=AKRDA5LYJ5NSRD2MI223UC3AB4Q24)

## Wall Follower

### TODO

## Person Follower

### TODO
