Now that we have real, interesting perceptual information to work with, we can [close the loop](https://en.wikipedia.org/wiki/Control_theory#Open-loop_and_closed-loop_(feedback)_control) and use this information to make Kuri look at a person. This kind of robotic control is sometimes referred to as [visual servoing](https://en.wikipedia.org/wiki/Visual_servoing)

## Make Kuri look at a person

Modify your `face_detection_demo.py` so that whenever a face is detected, it will attempt to center the detection in the middle of the frame.

You should be able to accomplish this with a very simple control policy. For instance, 

    if face_below_middle_of_frame:
        tilt_up()
    else if face_above_middle_of_frame:
        tilt_down()
    ...

* Your controller should stop at some point when the face is "close enough" to centered
* Don't worry too much about what to do when the robot can't see any faces at all. It can sit still in this case
* Consider implementing a simple [proportional controller](https://en.wikipedia.org/wiki/Proportional_control) if you find that a simple case-based policy doesn't work well. 