# crow_robot_msgs

The ROS2 interfaces used by the control [logic](https://gitlab.ciirc.cvut.cz/imitrob/common-imitrob-setup-hw-sw/crow/-/blob/main/src/crow_control/crow_control/logic.py) node to communicate with a robot node (example [dummy robot](https://gitlab.ciirc.cvut.cz/imitrob/common-imitrob-setup-hw-sw/crow/-/blob/main/src/crow_control/crow_control/testing/dummy_action_robot.py) node).

## Robot actions

There is only one type of action defined in this package:  
[RobotAction.action](action/RobotAction.action)  

### Requesting a robot action

This RobotAction interface is used to send commands to the robot. Different behavior is achieved by setting the **[RobotActionType](msg/RobotActionType.msg)  `robot_action_type`** field in the request part of the RobotAction. This field can have the following values:  
* int32 POINT         = 0  
  * The robot moves above the specified position and then returns home (default position).  
* int32 PICK_N_HOME   = 1  
  * The robot picks up an object from the specified position and then returns home.  
* int32 RELEASE       = 2  
  * The robot opens the gripper.  
* int32 PLACE_N_HOME  = 3  
  * The robot places the currently held object (goes to the specified position and opens the gripper) and then returns home.  
* int32 PICK_N_PASS   = 4  
  * The robot picks up an object at the specified position, goes to the specified location (part of the message) and waits for the user to take the object from the gripper (if "jolt" in the joints is felt as the user tries to pry the object from the gripper, the gripper should open a little).
* int32 PASS          = 5  
  * The same as PICK_N_PASS but without the "PICK" part - the object is already held in the gripper, so only "PASS" operation is executed.
* int32 PICK_N_PLACE  = 6  
  * Standard pick 'n' place operation - two poses are specified - one where an object is located and the other one where it should be placed.

  Even though the type of action is specified in the the request, the request also needs to be sent to a different end point ("action topic"). By default, these end points are used:
    * POINT = 'point'  
    * PICK_N_HOME = 'pick_n_home'  
    * RELEASE = 'release'  
    * PLACE_N_HOME = 'place_n_home'  
    * PICK_N_PASS = 'pick_n_pass'
    * PASS = 'pass'  
    * PICK_N_PLACE = 'pick_n_place'  

#### Other request fields  
* string frame_id  - coordinate frame (in which the poses are sent)
* int16 robot_id - ID of the robot on which the action should be executed (counts from 0; -1 means "any")
* geometry_msgs/Pose[] poses  - zero or more poses of target locations; depends on the action:  
    * POINT - one location (where to point)  
    * PICK_N_HOME - one location (object to pick up, home is defined by the robot)  
    * RELEASE - no locations  
    * PLACE_N_HOME - one location (location where to put the object)
    * PICK_N_PASS two locations (object to pick up and the "waiting" location, where the user can take the object)
    * PASS - one location (the "waiting" location)
    * PICK_N_PLACE - two locations (pick, place)  
* float64[3] size - the approximate object size (in each of the main axis)  
* ObjectType object_type - type of object to pick up (if any, see [ObjectType](msg/ObjectType.msg))
* Units request_units - units in which the positions are specified (in our case, always meters, see [Units](msg/Units.msg))

### Feedback 

During execution, the robot sends feedback in which phase of execution it is. The currently used phases are specified in [CoreActionPhase](msg/CoreActionPhase.msg) but this can be changed. It servers only as a feedback to see if anything is happening on robot side.

### Result  

After the end of an action, result of type [ActionResultFlag](msg/ActionResultFlag.msg) is sent by the robot to the control logic. This contains only one field: `int32 flag`. This flag only has a meaning if there is a failure and it is used to identify the cause of failure. The flag names (listed in [ActionResultFlag](msg/ActionResultFlag.msg)) should be descriptive enough to understand what they mean.


## Robot services

There is one robot service: [GetRobotStatus](srv/GetRobotStatus.srv) located at `get_robot_status` end point ("service topic").  

The request part specifies the ID of the robot of which status is requested.  

The response contains [RobotStatus](msg/RobotStatus.msg) `robot_status` field. This contains:
* robot_pose - poses of the robot joints (we currently don't use that, so it is irrelevant)  
* gripper_status - whether the gripper is opened or closed (this is important, we use this)
* robot_is_ready - whether the robot is ready to take the next command (i.e., idle)  
* accepted - whether the robot accepted the last command   
* done - whether robot finished executing the last command   
* fail - whether the robot fail the last command   
* not_feasible - whether the the last command is not feasible   

The last four flags are not used.