#!/usr/bin/env python3
# Students: [kang(kevin) wang] 
# Lab 4: Pick and_place
# Date: [11/02/2024]
# Acknowledgements: [moveit_tutorials/pick_place_tutorial]

from __future__ import print_function
from moveit_msgs.msg import Constraints, OrientationConstraint
import sys
import copy
from scipy.spatial.transform import Rotation as R

import rospy
import moveit_msgs.msg
from six.moves import input
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose

try:
    from math import pi
    from math import cos
    from math import tau
    from math import dist
    from math import fabs
except:  # For Python 2 compatibility
    from math import pi
    from math import cos
    from math import fabs
    from math import sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        ##
        ## We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        ## We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        ## We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        ## Sometimes for debugging it is useful to print the entire state of the
        ## robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.bin_name = ["caliper_bin", "mallet_bin", "pliers_bin", "screwdriver_bin"]
        self.battery_name = ["battery1"]

        self.tools_name = ["caliper","mallet", "pliers", "screwdrivers"]
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.batteries_loc = []
        self.bins_loc = []
        self.home_loc = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.tool_loc = []
        # self.mallet_loc = []
        # self.pliers_loc = []
        # self.screwdrivers_loc = []



    def home_position(self):
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = self.home_loc[0]
        joint_goal[1] = self.home_loc[1]
        joint_goal[2] = self.home_loc[2]
        joint_goal[3] = self.home_loc[3]
        joint_goal[4] = self.home_loc[4]
        joint_goal[5] = self.home_loc[5]
        joint_goal[6] = self.home_loc[6]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def pick_up_tools(self, tool):
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = tool[0]
        joint_goal[1] = tool[1]
        joint_goal[2] = tool[2]
        joint_goal[3] = tool[3]
        joint_goal[4] = tool[4]
        joint_goal[5] = tool[5]
        joint_goal[6] = tool[6]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def pick_up_batteries(self):
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = self.batteries_loc[0][0]
        joint_goal[1] = self.batteries_loc[0][1]
        joint_goal[2] = self.batteries_loc[0][2]
        joint_goal[3] = self.batteries_loc[0][3]
        joint_goal[4] = self.batteries_loc[0][4]
        joint_goal[5] = self.batteries_loc[0][5]
        joint_goal[6] = self.batteries_loc[0][6]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def drop_off_tools(self, tool):
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = tool[0]
        joint_goal[1] = tool[1]
        joint_goal[2] = tool[2]
        joint_goal[3] = tool[3]
        joint_goal[4] = tool[4]
        joint_goal[5] = tool[5]
        joint_goal[6] = tool[6]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def drop_off_batteries(self):
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = self.bins_loc[0][0]
        joint_goal[1] = self.bins_loc[0][1]
        joint_goal[2] = self.bins_loc[0][2]
        joint_goal[3] = self.bins_loc[0][3]
        joint_goal[4] = self.bins_loc[0][4]
        joint_goal[5] = self.bins_loc[0][5]
        joint_goal[6] = self.bins_loc[0][6]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -2.863
        joint_goal[1] = 0.157 
        joint_goal[2] = 2.894 
        joint_goal[3] = -2.004
        joint_goal[4] = -0.040 
        joint_goal[5] = 1.852
        joint_goal[6] = 0.831

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_joint_state2(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -2.857
        joint_goal[1] = -0.772 
        joint_goal[2] = -1.258 
        joint_goal[3] = -1.662
        joint_goal[4] = -0.736 
        joint_goal[5] = 1.422 
        joint_goal[6] = 1.897 

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.orientation.x = 1.0
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_batteries(self, timeout = 4):
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.05  # above the panda_hand frame
        box_pose.pose.position.x = 0.5  # above the panda_hand frame
        joint_state = [-0.024, 0.531, 0.068, -2.307, -0.114, 2.835, 0.917]
        self.batteries_loc.append(joint_state)

        scene.add_mesh(self.battery_name[0], box_pose, '/home/kwang/catkin_ws/src/pick_place_python/scripts/models/battery.stl', size = (1,1,1))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def add_tools(self, timeout = 4):
        scene = self.scene

        caliper_pose = geometry_msgs.msg.PoseStamped()
        caliper_pose.header.frame_id = "world"
        caliper_pose.pose.orientation.w = 1.0
        caliper_pose.pose.position.z = -0.05  # above the panda_hand frame
        caliper_pose.pose.position.y = -0.2  # above the panda_hand frame
        caliper_pose.pose.position.x = 0.45  # above the panda_hand frame
        joint_state = [-0.806, 0.419, 0.345, -2.378, -0.360, 2.725, -2.400]
        self.tool_loc.append(joint_state)
        scene.add_mesh(self.tools_name[0], caliper_pose, '/home/kwang/catkin_ws/src/pick_place_python/scripts/models/caliper_scaled.stl', size = (1,1,1))

        mallet_pose = geometry_msgs.msg.PoseStamped()
        mallet_pose.header.frame_id = "world"
        rotation = R.from_euler('y', 90, degrees=True)
        quaternion = rotation.as_quat()  # Returns (x, y, z, w)

        # Set orientation
        mallet_pose.pose.orientation.x = quaternion[0]
        mallet_pose.pose.orientation.y = quaternion[1]
        mallet_pose.pose.orientation.z = quaternion[2]
        mallet_pose.pose.orientation.w = quaternion[3]
        mallet_pose.pose.position.z = 0.05  # above the panda_hand frame
        mallet_pose.pose.position.y = -0.4  # above the panda_hand frame
        mallet_pose.pose.position.x = 0.3  # above the panda_hand frame
        joint_state = [-0.536, 0.357, -0.142, -2.577, 0.231, 2.925, 1.673]
        self.tool_loc.append(joint_state)
        scene.add_mesh(self.tools_name[1], mallet_pose, '/home/kwang/catkin_ws/src/pick_place_python/scripts/models/mallet_scaled.stl', size = (1,1,1))

        pliers_pose = geometry_msgs.msg.PoseStamped()
        pliers_pose.header.frame_id = "world"
        pliers_pose.pose.orientation.w = 1.0
        pliers_pose.pose.position.z = 0.05  # above the panda_hand frame
        pliers_pose.pose.position.y = -0.4  # above the panda_hand frame
        pliers_pose.pose.position.x = 0.5  # above the panda_hand frame
        joint_state = [-0.316, 0.858, -0.453, -1.664, 0.513, 2.401, 1.371]
        self.tool_loc.append(joint_state)
        scene.add_mesh(self.tools_name[2], pliers_pose, '/home/kwang/catkin_ws/src/pick_place_python/scripts/models/pliers_scaled.stl', size = (1,1,1))

        screwdriver_pose = geometry_msgs.msg.PoseStamped()
        screwdriver_pose.header.frame_id = "world"
        screwdriver_pose.pose.orientation.w = 1.0
        screwdriver_pose.pose.position.z = 0.05  # above the panda_hand frame
        screwdriver_pose.pose.position.y = -0.1  # above the panda_hand frame
        screwdriver_pose.pose.position.x = 0.6 # above the panda_hand frame
        joint_state = [1.093, 1.733, -1.513, -1.888, 1.743, 1.574, 1.589]
        self.tool_loc.append(joint_state)
        scene.add_mesh(self.tools_name[3], screwdriver_pose, '/home/kwang/catkin_ws/src/pick_place_python/scripts/models/screwdriver_scaled.stl', size = (1,1,1))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def setup_scene(self, timeout = 4):
        scene = self.scene
        bin_pose = geometry_msgs.msg.PoseStamped()
        bin_pose.header.frame_id = "world"
        # Orientation: 90 degrees rotation around the X-axis
        rotation = R.from_euler('x', 90, degrees=True)
        quaternion = rotation.as_quat()  # Returns (x, y, z, w)

        # Set orientation
        bin_pose.pose.orientation.x = quaternion[0]
        bin_pose.pose.orientation.y = quaternion[1]
        bin_pose.pose.orientation.z = quaternion[2]
        bin_pose.pose.orientation.w = quaternion[3]
        bin_pose.pose.position.z = 0.05  # above the panda_hand frame
        bin_pose.pose.position.y = 0.5  # above the panda_hand frame
        bin_pose.pose.position.x = 0.15  # above the panda_hand frame
        joint_state = [1.299, 0.793, -0.946, -2.065, 0.941, 2.346, 2.069]
        self.bins_loc.append(joint_state)

        scene.add_mesh(self.bin_name[0], bin_pose, '/home/kwang/catkin_ws/src/pick_place_python/scripts/models/bin_scaled.stl', size = (1,1,1))
        
        bin_pose.pose.position.z = 0.05  # above the panda_hand frame
        bin_pose.pose.position.y = 0.3  # above the panda_hand frame
        bin_pose.pose.position.x = 0.15  # above the panda_hand frame
        joint_state = [1.965, 0.387, -1.117, -2.286, 0.553, 2.405, 2.119]
        joint_state = [-1.099, -0.434, 2.007, -2.307, 0.603, 2.405, 1.260]
        # joint_state = [2.505, 1.243, -1.565, -2.317, 1.330, 1.796, 1.]
        self.bins_loc.append(joint_state)

        scene.add_mesh(self.bin_name[1], bin_pose, '/home/kwang/catkin_ws/src/pick_place_python/scripts/models/bin_scaled.stl', size = (1,1,1))
        
        bin_pose.pose.position.z = 0.05  # above the panda_hand frame
        bin_pose.pose.position.y = 0.5  # above the panda_hand frame
        bin_pose.pose.position.x = 0.4  # above the panda_hand frame
        joint_state = [-2.173, 0.052, 2.778, -2.890, 0.004, 2.858, 2.263]
        self.bins_loc.append(joint_state)

        scene.add_mesh(self.bin_name[2], bin_pose, '/home/kwang/catkin_ws/src/pick_place_python/scripts/models/bin_scaled.stl', size = (1,1,1))

        bin_pose.pose.position.z = 0.05  # above the panda_hand frame
        bin_pose.pose.position.y = 0.3  # above the panda_hand frame
        bin_pose.pose.position.x = 0.4  # above the panda_hand frame
        joint_state = [1.486, 1.3, -1.415, -1.474, 1.285, 1.694, 0.837]
        self.bins_loc.append(joint_state)

        scene.add_mesh(self.bin_name[3], bin_pose, '/home/kwang/catkin_ws/src/pick_place_python/scripts/models/bin_scaled.stl', size = (1,1,1))


        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

        

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.025  # above the panda_hand frame
        box_pose.pose.position.x = 0.5  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.25, 0.5, 0.5))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_battery(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        battery_name = self.battery_name[0]
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, battery_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def attach_tool(self, tool_name,  timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, tool_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_tool(self, tool_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=tool_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
    
    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def detach_battery(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        battery_name = self.battery_name[0]
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=battery_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

    def sort_tools(self):
        for i in range(4):
            self.pick_up_tools(tool=self.tool_loc[i])
            self.attach_tool(tool_name=self.tools_name[i])
            self.drop_off_tools(tool=self.bins_loc[i])
            self.detach_tool(tool_name=self.tools_name[i])
        # self.pick_up_tools(tool=self.tool_loc[1])
        # self.attach_tool(tool_name=self.tools_name[1])
        # self.drop_off_tools(tool=self.bins_loc[1])
        # self.detach_tool(tool_name=self.tools_name[1])
        self.home_position()
        return True

def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the Lab MoveGroup Python Interface")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input("============ Press `Enter` to begin the execution by setting up the moveit_commander ...")
        tutorial = MoveGroupPythonInterfaceTutorial()

        input("============ Press `Enter` to set up the scene...")
        # tutorial.setup_scene()

        # tutorial.add_batteries()
        # tutorial.add_box()
        # tutorial.add_box2()
        tutorial.setup_scene()
        tutorial.add_tools()
        # tutorial.add_box3()
        input("============ Press `Enter` to move the pen...")
        tutorial.sort_tools()
        # tutorial.pick_up_the_pen()
        # tutorial.pick_up_batteries()
        # tutorial.attach_battery()
        # tutorial.drop_off_batteries()
        # tutorial.detach_battery()
        # tutorial.home_position()

        print("============ Robot complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
