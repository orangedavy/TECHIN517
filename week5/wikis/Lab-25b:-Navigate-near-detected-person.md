## 3D face detection

In Lab 18b you used a face detector that gave you the location of the face in the 2D image. Next you will convert that into a 3D face pose using the size of the face as a proxy of the distance of the face from the robot. Your computation of the distance of the face does not need to be very accurate but we recommend collecting some examples of (distance, face size) pairs  by systematically varying distance to create your mapping from face size to distance. 

Once you know the distance of the face, you will now need to use transform arithmetic to convert the distance, pose of the robot head, and position of the face in the robot's image into a 3D pose of the face relative to the robot's base. To simplify you might want to keep servoing with the head to keep the face at the center of the camera, such that the direction of the face can be determined only based on the neck pose. You could further simplify by assuming a fixed height of the person and not worry about vertical centering of the face.

Modify your `face_detection_demo.py` to make the face pose computation described above and then publish the computed 3D face pose. We recommend also publishing a marker that can be visualized in RViz to enable testing of the 3D face detection algorithm.

## Using 3D face pose to determine a navigation target

Next, make the robot navigate in front of the detected person. You can trigger navigation through an interactive marker (e.g. right click on the detected face marker and choose from menu) or any other interface you prefer. You will need to transform arithmetic to combine the information about current pose of the robot, the 3D pose of the detected face, and the assumption that the detected person was looking towards Kuri in order to determine where Kuri should navigate to.



