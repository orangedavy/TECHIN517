Kuri's default behavior is to roam around and snap photos, so you can imagine that Mayfield invested quite a bit of thought and effort into implementing some nice [detectors](https://en.wikipedia.org/wiki/Object_detection). Amongst these out of the box detectors is a face detector which will likely come in handy when you implement interactions.

Note that these models are **only available on the real robot**. You won't be able to use the simulator to play with them. 

## Read Mayfield's Vision documentation

This is one area where the Kuri documentation does a pretty good job explaining things, so check out their page
on vision for the outline of how the system works.

[Mayfield vision documentation page](http://docs.freekuri.com/guides/advanced/vision.html)

## Test out face detection

* Create a script in applications called `face_detection_demo.py`. We've included all of the necessary message types in the course repo, so you should be able to run this node from the lab machine pointed at a robot master.
* Use the helpful `Vision` class in `robot_api` to activate the face_detector
* Register a callback on the `face_change` event and change the light color to visualize some aspect of the detections (e.g. confidence, number of faces, ID, size, position...)
* Use this demo to get a feel for how well the detector works. See if you can determine how far away a face has to be before it is no longer detectable, or other configurations that make it fail.