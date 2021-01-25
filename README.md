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
I structured my code following the object-oriented layout outlined during class. As such, I initialized the class by calling the publisher node "/cmd_vel" to gain access to the robot's linear and angular velocity: the linear velocity for moving the robot forward and the angular velocity for shifting directions. Moreover, I added in a 1 second pause to ensure the publisher is fully connected before publishing any data to it. <br /> <br /> 
For the run function, I created a while loop that instructs the robot to move forward for a moment and then turn counterclockwise by 90 degrees. To send the robot forward, a linear x rate of 0.1 is published to the publisher. After a 4 second interval, the linear velocity is halted, and the angular velocity is changed to 0.3980 to facilitate a 90 degree turn over the course of 4 seconds. This loop is repeated to send the robot on a square path. As mentioned earlier, the variable offset is the value added to the calculated angular velocity to compensate for the discrepancies in the stopping and accelerating mechanisms in the robot.
### Demo 
![Driving in a Square Demo](https://raw.githubusercontent.com/kiana1317/Warmup-Intro-to-Robotics/main/gifs/drivesquare.gif?token=AKRDA5LYJ5NSRD2MI223UC3AB4Q24)

## Wall Follower
### Description
The goal of this robot behavior was to get a robot located within a square room to follow the walls in the room continuously. To tackle this problem, the robot first drove forward to find a wall. Once the robot found a wall, it rotated counterclockwise until the robot was facing a new wall. Moreover, while driving towards the new wall small adjustments are made to the robot’s angular velocity to keep the robot running parallel to the wall. This process is repeated as the robot rounds the room.
### Code Structure
"init": Initializes the subscriber and publisher topics. Also, a twist object is created to update the velocity of the robot and a global variable to track whether the robot encountered a wall.
</br> </br>
"processScan": In part 1 of the function, the robot moves forward until it comes within 1 distance from a wall. In part 2, if the robot is within the turning distance, the robot rotates at a proportional control rate until the robot is facing an object further away than the turning point. If the robot is outside of the turning distance, it goes forward while receiving slight angular adjustments to keep it going straight.
</br></br>
### Demo
![Wall Follower Demo](https://raw.githubusercontent.com/kiana1317/warmup_project/main/gifs/wall_follower.gif)

## Person Follower
### Description
The goal of this robot behavior was to get a robot to follow an object located in an empty room. To achieve this, I determined if there was an object in the room and if so, I had the robot to turn and move in the object's direction.
### Code Structure
"init": Initializes the subscriber and publisher topics. Also, a twist object is created to update the velocity of the robot.
</br></br>
"processScan": In the function, first the minimum value in the scan ranges is found. Next, the function checks if the robot is within stopping range of the object. Following, the function determines if there are any objects placed in the space. If so, the function proceeds by iterating through the degrees in the scan ranges to find where the object is located. Depending on if the value lies to the left or right of the robot, the robot will turn towards the object and follow the person/object.
### Demo
![Person Follower Demo](https://raw.githubusercontent.com/kiana1317/warmup_project/main/gifs/person_follower.gif)
