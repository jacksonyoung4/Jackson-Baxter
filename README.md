# Jackson Baxter Guide
## Starting Robot
- Turn on at power point (yellow power cord)
- Open terminal
``` bash
cd Jackson/BaxterCurtin/ws
```
- Connect to baxter
``` bash
. baxter.sh
. devel/setup.sh
```
- Enable robot
``` bash
rosrun baxter_tools enable_robot.py -e 
```
- Untuck arms:
``` bash
rosrun baxter_tools tuck_arms.py -u 
```
- Launch action server to allow moveit to actually move robot
``` bash
rosrun baxter_interface joint_trajectory_action_server.py 
``` 
## Starting MoveIt
- Open a new terminal
``` bash
. baxter.sh
. devel/setup.sh
```
- Launch MoveIt
``` bash
roslaunch baxter_moveit_config demo_baxter.launch
```
## Starting 2F-85 Gripper
- Open a new terminal
``` bash
. baxter.sh
. devel/setup.sh
```
- Launch gripper
``` bash
roslaunch robotiq_2f_gripper_control robotiq_action_server.launch
```
## Testing Custom Script
- Open terminal
- Connect to baxter
``` bash
. baxter.sh
. devel/setup.sh
```
- Run script with commandline remap
``` bash
python go_to_pos.py /joint_states:=/robot/joint_states
```
- Run gripper script
``` bash
python gripper.py
```
## Shutting Down Robot
- Close RVIZ MoveIt window and ctrl + c in that terminal.
- ctrl + c in window where joint_trajectory_action_server.py was launched
- Tuck arms, untuck arms, and disable robot.
``` bash
rosrun baxter_tools tuck_arms.py -u 
rosrun baxter_tools tuck_arms.py -t 
rosrun baxter_tools enable_robot.py -d
```
