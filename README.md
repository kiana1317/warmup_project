# Warmup_project
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

## Other Questions
### Challenges
During this project, the main challenge I faced was developing a familiarity with ROS and its functions. Initially, I was unsure about how to properly interpret the results from the ROS topic scan, which made completing the Wall Follower implementation difficult. After realizing that the 360 ranges corresponded to 360 degrees, it made managing those values much more feasible. Moreover, I found navigating the noise in the robot’s movement a bit of a challenge. Given the noise, it was difficult to predict the turning rate and resulting angle for the robot, especially in the drive in a square implementation. To resolve this issue, I just adjusted the linear and angular velocity to account for the noise and utilized the proportional control function discussed in class.
### Future Improvements
For the drive-in square behavior, I would test out more ways to make sure the behavior forms a perfect square. The current implementation works for a few rounds, but after a bit, one could notice how the robot drifts off its path. For the wall follower, I would give more distance between the wall and the robot. Also, I would investigate having the robot function both counterclockwise and clockwise. For the person follower, I currently have the robot run towards the nearest object it sees rather the most optimal spot of the object. Therefore, I would probably adjust the robot to move towards the center of the object rather than its side.
### Key Takeaways
* Cmd_vel and scan are crucial topics for managing a robot’s behavior – As seen in the projects, using the linear and angular velocity from the /cmd_vel topic dictates a robot’s behavior. Moreover, /scan reads the room and helps the robot to navigate any space.
* Use proportional control whenever possible – These projects have shown that using proportional control works to mitigate the noise in the robot’s velocity and orientation.

