Because the class is sharing a single Kuri, you should strive to use the simulator as much as possible. But the simulator doesn't model Kuri's chest lights, so for this lab you'll _have_ to test your code on the robot.

In this lab, we will test whether you are correctly publishing a message to the light control topic.

# Tips for working with Kuri

Kuri wasn't designed as a research platform, or really as a platform that anyone besides Mayfield would use.
There isn't a lot of public documentation about the robot, so we've put some pages on wiki that hit
the some usage highlights. You may need to refer to it

[Usage highlights](https://github.com/hcrlab/wiki/wiki/Robots:-Kuri:-Usage)

# Robot etiquette

* Claim the robot by lifting it and moving it near your lab machine
* Do not use the robot for more than 90 minutes at a time
* Do not leave your code running on the robot when you are not using it
* Put the robot on the charger when you're done

There are six teams and one robot, so you must be courteous and try to use the simulator as much as possible.
You must acquire confidence in your code before running on the robot, and you must have a short, focused plan for what you want to test on the real robot.
As soon as you are done with the robot, you must allow others to use it as soon as possible.

Some general ways to test your code without the real robot:

* Save several camera images from the real robot and hallucinate them in simulation
* Spawn relevant objects in Gazebo
* Write [unit tests](http://wiki.ros.org/UnitTesting)
* Manually inspect values of messages being published or goals being sent using `rostopic echo`

# Running code on the real robot
ROS nodes will search for a ROS master using the environment variable `ROS_MASTER_URI`.
When you run `roscore` and the simulator, you are creating a ROS master at localhost:11311.
We have configured the course computer's .bashrc files to show the hostname of the ROS_MASTER_URI in the terminal prompt:
```
[robonaut ~/catkin_ws/src/cse481c (localhost)]$ 
```

To have your ROS nodes search for a different ROS master, use the `setrobot` command:
```
[robonaut ~/catkin_ws/src/cse481c (localhost)]$ setrobot panang
[robonaut ~/catkin_ws/src/cse481c (panang)]$
```

`setrobot` is a convenience command we added to your .bashrc files to set the `ROS_MASTER_URI`.
If you run a ROS node or use a tool like `rostopic list`, it will now contact the robot's ROS master at astro:11311.
Any messages you publish or services you call, or actions you run will now communicate with the robot instead of your simulator.

**Note:** You must run `setrobot` in each terminal window that you want to communicate with the robot.

Once you are done using the robot, you can point your `ROS_MASTER_URI` back to your simulator with:
```
setrobot sim
```

# Discovering the relevant topic

How do we change the colors of the LEDs? We'll do some sleuthing on the robot to find out.
Point your environment at Kuri using the `setrobot` utility, then start poking around with

    rostopic list

See anything promising? Try filtering things with `grep`

    rostopic list | grep "something"

You can then use `rostopic info /topic_name` to see what the message type is.

Once you're confident you know the topic, you can use `rostopic pub /topic_name` with tab completion to help you quickly send a test message.

# Finish the wrapper class

We've left the outline of a wrapper class in `robot_api`. Using the knowledge you gained from above, finish this class.

# Test your wrapper class

Fill out `lights_demo.py`. Make the `on` command turn the lights into some interesting configuration, and the `off` command turn them all off.

# Test your light code

1. Make sure your code at least runs without crashing on your machine
1. Use `rostopic echo` to verify that the message you are publishing looks the way you expect
1. Position the robot's gripper so that it is about to pick up the turtle.
1. `setrobot panang`
1. `rosrun applications light_demo.py on`
1. The lights should change (you choose how!)
1. `rosrun applications light_demo.py off` 
1. All lights should turn off.
1. `setrobot sim`