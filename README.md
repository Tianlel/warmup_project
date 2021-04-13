# warmup_project

## Driving in a square 
I use the timing method to let the robot to move in a square once. I use the Twist message to instruct the robot to move forward for a fixed time, then turn for a fixed time such that the total turn roughly matches 90 degree, and repeat this sequence for 4 times.  

`init`: initializes a ros node, set the linear and angular speed and move time respectively, and initializes a Twist message.

`run`: instructs the robot to move forward and turn 90 degree for 4 times, and eventually stops the robot.

![a](gif/drive_square.gif)

## Person follower
The robot drives towards the person and stops in front of him (facing the person) using PID.   

`init`: initializes a ros node, set up publisher and subscriber, and initializes a Twist message.

`process_scan`: finds the angle at which the robot is closest to the person, using PID, turn the robot towards this angle and drives it towards the person, stops the robot if it is very close to the person.

`run`: keeps the program running

![b](gif/person_follower.gif)

## Wall follower
The robot first navigates to a wall and then moves along the wall at an approximately fixed distance. 

`init`: initializes a ros node, set up publisher and subscriber, initializes a Twist message, and initializes `self.find_wall=0` to indicate that the robot has not been navigated up to a wall.

`process_scan`: when the program starts, first navigate the robot up to a wall. Then instructs the robot to stop and turn left whenever it detects a wall in front of it; if it is not facing a wall, drive it along the wall while maintaining distance by keeping the minimum distance on its right to be at 90 degree using PID. 

![c](gif/wall_follower.gif)


## Challenges
1. In my initial implementation of the **driving in a square** project, the first message couldn't be published. It turned out that it takes time for the publisher to be successfully set up and adding `rospy.sleep(1)` after the initialization of the publisher fixes this. 
2. The **wall follower** project was really challenging for me because I spent a long time trying to figure out how to implement different cases in the call_back function. The part that instructs the robot to navigate to a wall only needs to be executed once while the drive-along-the-wall part keeps looping. That's why I set up a public variable to check whether the first step has been performed. Figuring out what conditions to check for boundary cases was also quite frustrating for me because many of the approaches I came up got really complicated with noise and required many parameter tweeking. There were also other approaches, such as using timing to turn the robot, that I couldn't really figure out how to implement as part of the call_back function. 

## Future work
I would like to add more accurate control for the wall-follower such that the robot won't stop in the middle while it is driving along the wall and that it could maintain a more fixed distance from the wall.

## Takeaways
1. **Plan and draw a diagram before coding**: I found that it was really easy for me to loose track of my thought if I start coding without writing down an outline on paper. Even though I thought I had a clear picture in my mind, the workflow could become really nasty when I ran into bugs and especially scenarios that were complicated by noises. That was why I spent a huge chunk of time on the wall-follower project. 
2. **Deal with noises/odd behaviors last**: I think I would have a more efficient workflow if I had focused on getting the key structures of code implemented first and then deal with odd behaviors. In this project, I kept getting distracted by the noises and odd behaviors and tried to deal with these only to eventualy realize that they were caused by gazebo simulator glitches or not relevant to final product. 
