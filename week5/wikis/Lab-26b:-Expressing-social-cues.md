## Social cues API 
You will create new services and actions in the robot API to make Kuri express different emotions and perform communicative actions. Implement the following:
* `nod_head` an action that makes the head move up and down a few times as if to say yes.
* `shake_head` an action that makes the head side-to-side a few times as if to say no.
* `happy` a service that changes the robot's expression to look happy by setting the eyelid pose and belly LED color accordingly and playing an appropriate sound once when the state change happens.
* `sad` same as happy, but for expressing sadness.
* `neutral` same as happy and sad, but for getting the robot back to a neutral state, without making any sounds.

## Putting it all together

To complete Assignment 5 you will inject emotional expressions into the navigation demo you created in Lab 25b. Make the robot look sad when it does not see anyone, happy when it detects and localizes a person. The robot should shake its head when it receives the command to start navigating towards the person and go back to a neutral state while navigating.