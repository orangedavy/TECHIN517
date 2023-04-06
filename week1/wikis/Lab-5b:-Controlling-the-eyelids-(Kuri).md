# actionlib
You must be familiar with [actionlib](http://wiki.ros.org/actionlib) and the [actionlib](http://wiki.ros.org/actionlib/Tutorials) tutorials before starting.

# Trajectories
There is no documentation about Kuri's joints, but they actually work a lot like Fetch's. If you read the Fetch docs on the [arm and torso](http://docs.fetchrobotics.com/api_overview.html#arm-and-torso), you will see that to control the torso, you must send a *trajectory* as opposed to a position to lift the torso to. Kuri's eyelids work the same way.

A trajectory is simply a list of (position, time) pairs, which specifies a motion with multiple in-between points.
A trajectory can also specify the desired velocity and acceleration at each point of time, which can result in smoother motions.
Generally, the robot's controllers will automatically interpolate trajectories for you, so you only need to send the endpoint.
This works well for Kuri's joints, but may not work well for complicated mechanisms like arms.

# Eyelid trajectories
The robot's controllers will enforce some constraints on your trajectories (joint limits, acceleration limits).

As a result, all you have to do to control the eyelids is to send a trajectory with a single point: the end point.
The end point will be the desired eyelid position, and the desired time to reach that height can be whatever you like (though the controller will clamp you into reasonable limits).

# Write the head class
As in the previous labs, we have implemented a skeleton of a `Head` class for you in `robot_api/src/robot_api/head.py`.

Fill in the blanks. This _will_ be tricky. You'll have to use your sleuthing skills on the robots to explore the controller configuration. You will also need to refer to the ROS documentation to uncover the correct types for interacting with the action server clients for the controllers.

**Hints:**
* As discussed earlier, our "trajectory" only has one JointTrajectoryPoint: the end point.
* Some of the Fetch specific tutorials contain spoilers (uber hints). If you tire of the joys of exploration, check them for helpful guidance.

# Finish the demo
We have partially implemented a demo file for you.

Finish the implementation of the `eyes` command.

You should now be able to run the following and see the robot's eyes move up and down:
```
rosrun applications head_demo.py eyes .1
rosrun applications head_demo.py eyes .4
rosrun applications head_demo.py eyes .16
```