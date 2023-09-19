# Computational Robotics 2023: Warm-Up Project

## 1 Problem Statement

### 1.1 Tele-Op

Our first task was to make a keyboard-based controller for the robot. The goal was to press buttons on our keyboard, and have the robot respond with forward, backward, spin left, spin right, or stop depending on the button pressed.

### 1.2 Drive in a Square

Our second task was to have the robot drive in a 1 meter by 1 meter square. Precision was not a high priority for this step, so a ballpark square would work fine.

### 1.3 Wall Following

Our third task was to have the robot follow a wall using information from its LiDAR scanner. For the purposes of this warm up project, we decided to do this on one side of the robot, and assume the wall was smooth (no corners).

### 1.4 Person Following

Our fourth task was to have the robot follow a person. Here, a person is defined as any object reasonably close in front of the robot. The robot should turn and move towards that object.

### 1.5 Obstacle Avoidance

Our fifth task was to have the robot avoid obstacles it sees with the LiDAR. The goal was to have the robot move forward, but turn away at the sight of any obstacles.

### 1.6 State Controller

Our sixth and final task was to combine two or more previous tasks using a state controller.

## 2 Solution

We structured our code so that each individual part would have its own python file containing everything ROS2 needs to run. For parts 1-5, we defined a single `Node` object in each file and ran it. Part 6 needed multiple `Node`s.

### 2.1 Tele-Op

For Tele-Op, we created a node that ran a function on a timer. That function waits for a key to be pressed, then depending on the key press, sends a different `Twist` message to the robot. Unfortunately, our `get_key` function is blocking, which means that it will wait for a key press instead of exiting if no key is currently pressed. We recognize this is bad practice for ROS, however it worked for the purposes of this part and fixing it was outside the scope of the project.

### 2.2 Drive Square

Our drive square node implemented a rudimentary state machine with two states: turn and drive. Both of these states are time-based, and so will transition to the other after a certain amount of time. After 8 states have completed, the robot will have done a full square and the node will stop itself. This is not a particularly precise method, but worked for our purposes.

### 2.3 Wall Following

We implemented a node to have the robot drive parallel to a wall. To do this, we listened for a lidar scan and took two distance measurements, each 45 degrees off of due right. If the robot was perfectly parallel to the wall, these values would be equal. If it was turned towards the wall, the forward ray would be shorter, and the opposite if it was turned away. Using the difference between the two distances, we were able to make a Proportional controller to control the robot’s angular velocity while it was driving forward, keeping it parallel to the wall.

We encountered an issue that if the robot was turned too far away from the wall, the LiDAR would return a distance of 0, and the robot would spin the wrong way, making the situation worse. We solved this by setting any zeros to tens, which worked beautifully. It also has the added effect that the wall follower can properly deal with corners!

### 2.4 Person Following

For person following, we analyzed the LiDAR scan to find the “center of mass” for objects in front of the robot. After receiving a new LiDAR scan, we filtered out all points that were too far away, along with anything not in front of the robot. We then took the average angle and radius of the remaining points, and used that as the approximate center of mass. While this is not the mathematically perfect center of mass, it worked well enough. The robot would then turn to approximately that angle and approach that point, slowing down as it got close. Chaining this together over time means that the robot will follow a person in front of it.

### 2.5 Obstacle Avoidance

To implement obstacle avoidance, we used gradient descent. Each time we got a LiDAR scan, we converted everything to cartesian coordinates. We then calculate an environment function, where each point that the scan detected would be a “source,” and a point a few meters in front of the robot would be a “sink.” Each of the sources contribute a value of one over distance squared towards the environment, and the sink contributes a value of negative one over distance squared. We then take the derivative of this function at the point the robot is at, and the resulting value, when converted back to polar coordinates, gives an angle and radius that would be ideal for the robot to take. We then use a proportional controller for both forward speed and angular velocity. With proper tuning, this proves an effective method for obstacle avoidance.

### 2.6 Finite State Controller

For our finite state controller, we decided to switch between wall following, person following, obstacle avoidance, and a little dance, depending on a keypress on the keyboard. For this structure, we created a subclass of each of the nodes, which would listen on the `action` topic for a specific word, such as `”avoid”` or `”dance”`. The node corresponding to that action would activate when its word is published, and every other node would deactivate. We also created a `StateManager` node with the sole purpose of listening to the keyboard presses, and publishing the corresponding string to the `action` topic.

## 3 Challenges

We faced a few major challenges while working through these behaviors. The first was finding that the getKey function used for teleop purposes was blocking, meaning that it waits for a key to be pressed before continuing with code and even stopping running code prematurely.
Our next big challenge came from identifying what a person was. While implementing person_follower, our ankles were often only found as 2-5 data points. We were unsure for some time whether this was a code error or a lidar error, and ended up overcoming the issue by visualizing the neato’s surroundings using rviz.
Our final challenge was attempting to create a telop based system for our finite state controller. We attempted to create this system in a few ways, each of which ending with failure and half an hour less to finish this project. These include python imports, ros launch files, threading, and getKey blocking continuing to cause issues.

## 4 Future Work

With more time to work on this project, we would take on some of the challenges that prevented our finite state controller from functioning. We would meet with the project professor to find the error within getKey and then proceed to take a deeper dive into launch files and how we could implement them to get a cleaner overall code. 
Our other idea we wish we had the time to dive into was to combine two of our existing automatic behaviors and switch between them based on lidar scan data. The dance behavior was created for this very purpose. The original plan was for the neato to person follow until they get close enough that it has “found” the person, at which point it will celebrate with a dance.

## 5 Key Takeaways
### Take baby steps instead of setting lofty goals.
We jumped into a fairly ambitious finite state machine and paid for it. If we had instead opted for something closer to minimum viable product we would have come out of this with a working system and from there could have moved on to stretch goals. 
### Be more deliberate with scheduling.
We started out great on this project with the first few behaviors going relatively smoothly. After the mid project checkpoint we started to hit bigger issues that really required the help of a CA or professor. The unfortunate thing is that we would always find ourselves meeting at the tail end of or after office hours and had to stop without resolving these issues until the next day of class.
### DATA TYPES
This project really showed us the importance of the different data types in ROS2. Interpreting errors can be tricky with ROS2 and the new types that are new to us. This project forced us to be very particular with our types and gave us a lot of experience debugging which will definitely come in handy moving forward.
