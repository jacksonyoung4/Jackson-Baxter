# General libraries
import sys
import rospy
import actionlib

# For file saving
import os
import re

# Import MoveIt libs
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

# Initialize the MoveIt! commander and rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('baxter_move_test', anonymous=True)

# Instantiate RobotCommander (interface to the robot)
robot = RobotCommander()

# Instantiate PlanningSceneInterface (interface to the world)
scene = PlanningSceneInterface()

# Group for the left arm
left_arm_group = MoveGroupCommander("left_arm")

# Set the reference frame
left_arm_group.set_pose_reference_frame("base")
left_arm_group.set_planner_id("RRTConnectkConfigDefault")

moveit_tolerance = 0.01
left_arm_group.set_goal_tolerance(moveit_tolerance)

def get_next_filename(directory: str) -> str:
    if not os.path.exists(directory):
        os.makedirs(directory)

    existing_files = os.listdir(directory)
    numbered_files = [f for f in existing_files if re.match(r'\d+\.txt$', f)]

    if not numbered_files:
        return os.path.join(directory, "1.txt")

    numbers = [int(re.match(r'(\d+)\.txt$', f).group(1)) for f in numbered_files]
    next_number = max(numbers) + 1
    return os.path.join(directory, f"{next_number}.txt")

#def move_to_pose(move_group: MoveGroupCommander, pose: Pose):
   # """
   # Moves the specified MoveGroup to the given cartesian pose.
   # :param move_group: MoveGroupCommander for the arm
   # :param pose: Pose to move to
   # """
   # move_group.set_pose_target(pose)
   # move_group.go(wait=True)
   # #plan = move_group.go(wait=True)
   # move_group.stop()
   # move_group.clear_pose_targets()

if __name__ == "__main__":
    current = left_arm_group.get_current_pose()
    rospy.sleep(2.0)
    print(current)

    save_path = get_next_filename("baxter_poses")

    with open(save_path, "w") as f:
        f.write("PoseStamped:\n")
        f.write(f"  frame_id: {current.header.frame_id}\n")
        f.write("  position:\n")
        f.write(f"    x: {current.pose.position.x}\n")
        f.write(f"    y: {current.pose.position.y}\n")
        f.write(f"    z: {current.pose.position.z}\n")
        f.write("  orientation:\n")
        f.write(f"    x: {current.pose.orientation.x}\n")
        f.write(f"    y: {current.pose.orientation.y}\n")
        f.write(f"    z: {current.pose.orientation.z}\n")
        f.write(f"    w: {current.pose.orientation.w}\n")

    print(f"Pose saved to {save_path}")
    #current.pose.position.z += 0.1 
    #left_arm_group.set_pose_target(current)
    #left_arm_group.go(wait=True)
    #left_arm_group.stop()
    #left_arm_group.clear_pose_targets()
    moveit_commander.roscpp_shutdown()
    sys.exit(0)

    