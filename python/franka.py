#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Modified by Xiaoxiao Zhang to work with RESTful API

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import actionlib
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryGoal

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes


MOVEIT_ERROR_DECODE = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED: planner found no valid plan",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN: plan has issues (self-collision, joint limits)",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: scene changed during planning",
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED: trajectory execution failed (reflex? controller? FCI?)",
    MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "UNABLE_TO_ACQUIRE_SENSOR_DATA",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.PREEMPTED: "PREEMPTED: another goal replaced this one",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION: robot already in collision",
    MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "START_STATE_VIOLATES_PATH_CONSTRAINTS: current orientation outside constraint tolerance",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION: target pose collides with scene",
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
    MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
    MoveItErrorCodes.INVALID_OBJECT_NAME: "INVALID_OBJECT_NAME",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
    MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "COLLISION_CHECKING_UNAVAILABLE",
    MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
    MoveItErrorCodes.SENSOR_INFO_STALE: "SENSOR_INFO_STALE",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION: target is reachable in position but orientation can't be achieved",
}


def decode_moveit_error(error_code):
    """error_code may be a MoveItErrorCodes msg or an int."""
    val = getattr(error_code, "val", error_code)
    return MOVEIT_ERROR_DECODE.get(val, f"UNKNOWN_ERROR_{val}")


# Panda joint limits (rad) from franka_description URDF. Used by
# check_joint_limits() as a pre-flight safety check.
PANDA_JOINT_LIMITS = [
    (-2.8973,  2.8973),   # joint1
    (-1.7628,  1.7628),   # joint2
    (-2.8973,  2.8973),   # joint3
    (-3.0718, -0.0698),   # joint4
    (-2.8973,  2.8973),   # joint5
    (-0.0175,  3.7525),   # joint6
    (-2.8973,  2.8973),   # joint7
]


def check_joint_limits(joint_values, margin_rad=0.02):
    """Return list of dicts describing joints within margin_rad of a limit.
    Empty list means all joints are a safe distance from limits.

    Default margin = 0.02 rad (~1.15 deg). This is calibrated against the
    observed Franka firmware behavior: the low-level controller starts
    refusing motion commands at ~0.0005 rad (0.03 deg) from a limit. 0.02
    rad gives a ~30x safety buffer while still allowing routine motions
    and post-push states (where a joint may be 1-5 deg from a limit) to
    pass through."""
    bad = []
    for i, q in enumerate(joint_values):
        if i >= len(PANDA_JOINT_LIMITS):
            break
        lo, hi = PANDA_JOINT_LIMITS[i]
        min_dist = min(q - lo, hi - q)
        if min_dist < margin_rad:
            bad.append({
                "joint": i + 1,
                "current_rad": round(q, 4),
                "limit_lo_rad": lo,
                "limit_hi_rad": hi,
                "distance_to_nearest_limit_rad": round(min_dist, 4),
                "distance_to_nearest_limit_deg": round(min_dist * 57.2958, 2),
            })
    return bad


def check_trajectory_for_limits(plan, margin_rad=0.02):
    """Inspect every waypoint in a RobotTrajectory for joints too close to
    their limits. Returns list of {waypoint_index, joint, ...} dicts. Empty
    list means the trajectory is safe to execute.

    Use this AFTER planning and BEFORE executing, so that trajectories that
    would drift a joint into Franka's limit-refusal zone are caught at plan
    time rather than failing silently at the firmware layer."""
    violations = []
    if plan is None:
        return violations
    traj = getattr(plan, "joint_trajectory", None)
    if traj is None:
        return violations
    points = getattr(traj, "points", [])
    for i, point in enumerate(points):
        positions = getattr(point, "positions", None)
        if positions is None:
            continue
        bad = check_joint_limits(positions, margin_rad=margin_rad)
        for b in bad:
            violations.append({"waypoint_index": i, **b})
    return violations


def trajectory_joint_travel(plan):
    """Cumulative absolute travel of each joint over a RobotTrajectory, in
    radians: {joint_name: sum(|q[i+1]-q[i]|)}. Cumulative (not max-min) so a
    joint that swings out and back is counted. Empty dict if plan is empty."""
    travel = {}
    traj = getattr(plan, "joint_trajectory", None) if plan is not None else None
    if traj is None:
        return travel
    names = list(getattr(traj, "joint_names", []))
    points = getattr(traj, "points", [])
    prev = None
    for point in points:
        pos = getattr(point, "positions", None)
        if not pos:
            continue
        if prev is not None:
            for name, a, b in zip(names, prev, pos):
                travel[name] = travel.get(name, 0.0) + abs(b - a)
        prev = pos
    return travel


def is_user_stop_abort(error_text):
    """Detect Panda hardware user-stop button activation. libfranka emits
    `Move command aborted: User Stop pressed!` when the physical safety
    button is depressed. No software action clears it -- the user must
    release the button on the hardware."""
    if not error_text:
        return False
    lower = error_text.lower()
    return "user stop" in lower or "user_stop" in lower


def is_external_force_abort(error_text):
    """Best-effort: did this reflex fire because of external force (user hand,
    unexpected contact) rather than a controller-internal violation?

    Franka labels external-force reflexes as `cartesian_reflex` or
    `joint_reflex`. Limit/profile violations like
    `joint_motion_generator_position_limits_violation` are controller-internal
    and are usually worth auto-retrying after /recover.
    """
    if not error_text:
        return False
    lower = error_text.lower()
    return "cartesian_reflex" in lower or "joint_reflex" in lower


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
        # disable_signals=True: don't let rospy install its own SIGINT handler,
        # which otherwise swallows Ctrl-C and leaves the Flask server hanging.
        # Shutdown is handled in restful.py's __main__ (hard process-group kill).
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True,
                        disable_signals=True)

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
        group_name = "panda_manipulator"
        hand_group_name = "panda_hand"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # ---- 给关节3加一个软限制：比如 -2 ~ +2 rad ----
        from moveit_msgs.msg import Constraints, JointConstraint
        soft_limits = Constraints()

        jc2 = JointConstraint()
        jc2.joint_name = "panda_joint2"
        jc2.position = 0.0           # 以 0 为中心
        jc2.tolerance_above = 0.6    # 上面  +2 rad
        jc2.tolerance_below = 0.6    # 下面  -2 rad
        jc2.weight = 1.0             # 权重 1

        soft_limits.joint_constraints.append(jc2)


        hand_group = moveit_commander.MoveGroupCommander(hand_group_name)
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.hand_group = hand_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        # Set by stop() to signal in-flight motion loops not to auto-retry.
        # Cleared by plan_and_execute_with_retry at start of each motion.
        self._stop_requested = False
    def recover(self, wait=True, timeout=rospy.Duration(15.0)):
        """
        Trigger Franka automatic error recovery once.
        - wait: block until done
        - timeout: rospy.Duration, default 15 s. Previously defaulted to None (indefinite wait).
        Returns True on success, False on server-unavailable or result timeout.
        """
        action_name = "/franka_control/error_recovery"
        client = actionlib.SimpleActionClient(action_name, ErrorRecoveryAction)
        rospy.loginfo(f"Waiting for {action_name} action server...")
        if not client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr(f"{action_name} not available after 10s")
            return False
        rospy.loginfo("Franka error recovery server ready.")
        goal = ErrorRecoveryGoal()  # empty goal
        client.send_goal(goal)

        if wait:
            ok = client.wait_for_result(timeout)
            if not ok:
                rospy.logerr(f"{action_name} did not return within {timeout.to_sec()}s")
            return bool(ok)
        return True
    
    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group
        hand_group = self.hand_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        hand_goal = hand_group.get_current_joint_values()
        print(hand_goal)
        hand_goal[0] = 0.01
        hand_goal[1] = 0.01
        hand_group.go(hand_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_gripper_state(self, width):

        #max 0.039893
        if width > 0.032:
            width = 0.032
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        hand_group = self.hand_group

        hand_goal = hand_group.get_current_joint_values()
        print(hand_goal)
        hand_goal[0] = width
        hand_goal[1] = width
        hand_group.go(hand_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        hand_group.stop()

        ## END_SUB_TUTORIAL

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

    def go_to_pose_goal_xy(self, x, y):
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
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = 0.4

        pose_goal.orientation.x = 1.0
        pose_goal.orientation.y = 0.0
        pose_goal.orientation.z = 0.0
        pose_goal.orientation.w = 0.0

        move_group.set_pose_target(pose_goal)
        move_group.set_max_velocity_scaling_factor(0.05)
        move_group.set_max_acceleration_scaling_factor(0.05)
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

    def nudge_joint(self, joint_index, delta_rad):
        """Minimal-motion recovery: move one joint by delta_rad, keep all
        others at their current values. Intended for unsticking a joint that
        has drifted too close to its limit, where a full home move fails
        because the Franka controller refuses to execute while a joint is in
        the limit-proximity zone. A tiny, targeted motion sometimes gets
        through where a large multi-joint motion doesn't.

        joint_index: 0-based (0..6)
        delta_rad: signed displacement. Positive moves toward +limit.

        Returns dict similar to go_home: planner_tried, plan_error_codes,
        executed, planner_used, planning_time.
        """
        move_group = self.move_group
        move_group.clear_pose_targets()
        move_group.clear_path_constraints()
        move_group.set_num_planning_attempts(10)
        move_group.set_planning_time(5.0)

        joint_goal = list(move_group.get_current_joint_values())
        if not (0 <= joint_index < len(joint_goal)):
            return {"error": f"joint_index {joint_index} out of range", "executed": False}
        joint_goal[joint_index] += delta_rad

        # Clamp to valid range to avoid planning an infeasible goal
        if joint_index < len(PANDA_JOINT_LIMITS):
            lo, hi = PANDA_JOINT_LIMITS[joint_index]
            safety_margin = 0.01
            joint_goal[joint_index] = max(lo + safety_margin,
                                          min(hi - safety_margin, joint_goal[joint_index]))

        result = {"planner_tried": [], "plan_error_codes": [], "executed": False,
                  "target_joint": joint_index + 1, "target_value_rad": round(joint_goal[joint_index], 4)}
        try:
            move_group.set_joint_value_target(joint_goal)
            for planner_id in ("RRTConnect", ""):
                move_group.set_planner_id(planner_id)
                result["planner_tried"].append(planner_id or "(default)")
                plan_success, plan, planning_time, error_code = move_group.plan()
                result["plan_error_codes"].append(decode_moveit_error(error_code))
                if plan_success:
                    result["planner_used"] = planner_id or "(default)"
                    result["planning_time"] = planning_time
                    plan = move_group.retime_trajectory(
                        moveit_commander.RobotCommander().get_current_state(),
                        plan,
                        velocity_scaling_factor=0.15,
                        acceleration_scaling_factor=0.05,
                    )
                    exec_ok = move_group.execute(plan, wait=True)
                    result["executed"] = bool(exec_ok)
                    break
        finally:
            move_group.stop()
        return result

    def clear_stop(self):
        """Clear the user-stop flag so a new (multi-step) motion routine can run.
        Mirrors what plan_and_execute_with_retry does at the start of a motion."""
        self._stop_requested = False

    def stop_requested(self):
        """True if /control/stop was called and not yet cleared. Multi-step
        routines (e.g. payload identification) poll this to bail out early."""
        return self._stop_requested

    def move_to_joint_config(self, joint_goal, vel=0.1, accel=0.05):
        """Plan+execute a joint-space move to an explicit 7-vector joint goal.
        Used by payload identification to revisit identical configurations in
        the empty and loaded passes. Each joint is clamped into its safe range;
        tries RRTConnect then the default planner. Slow by default (vel/accel
        scaling) for quasi-static measurement. Returns a dict with executed,
        planner_used, planning_time, joint_goal, plan_error_codes, and (if a
        user-stop is active) stopped=True with no motion."""
        # Honor a user-stop: do not start a new move while a stop is active.
        if self._stop_requested:
            return {"executed": False, "stopped": True,
                    "joint_goal": list(joint_goal),
                    "planner_tried": [], "plan_error_codes": []}
        move_group = self.move_group
        move_group.clear_pose_targets()
        move_group.clear_path_constraints()
        move_group.set_num_planning_attempts(10)
        move_group.set_planning_time(5.0)
        move_group.set_max_velocity_scaling_factor(vel)
        move_group.set_max_acceleration_scaling_factor(accel)

        goal = list(joint_goal)
        for i in range(min(len(goal), len(PANDA_JOINT_LIMITS))):
            lo, hi = PANDA_JOINT_LIMITS[i]
            margin = 0.02
            goal[i] = max(lo + margin, min(hi - margin, goal[i]))

        result = {"planner_tried": [], "plan_error_codes": [], "executed": False,
                  "joint_goal": [round(v, 5) for v in goal]}
        try:
            move_group.set_joint_value_target(goal)
            for planner_id in ("RRTConnect", ""):
                move_group.set_planner_id(planner_id)
                result["planner_tried"].append(planner_id or "(default)")
                plan_success, plan, planning_time, error_code = move_group.plan()
                result["plan_error_codes"].append(decode_moveit_error(error_code))
                if plan_success:
                    result["planner_used"] = planner_id or "(default)"
                    result["planning_time"] = planning_time
                    result["executed"] = bool(move_group.execute(plan, wait=True))
                    break
        finally:
            move_group.stop()
            move_group.clear_pose_targets()
        return result

    def go_home(self):
        """Plan and execute a joint-space move to a canonical safe pose.

        Tries multiple planners in order (PTP, RRTConnect) since the active
        planner after plan_joint_path is LIN which can't plan to joint-space
        goals. Returns a dict with details so the caller can report why it
        failed if it did.
        """
        move_group = self.move_group
        move_group.clear_pose_targets()
        move_group.clear_path_constraints()
        move_group.set_num_planning_attempts(10)
        move_group.set_planning_time(5.0)
        move_group.set_max_velocity_scaling_factor(0.15)
        move_group.set_max_acceleration_scaling_factor(0.05)

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = -0.785
        joint_goal[2] = 0.0
        joint_goal[3] = -1.571
        joint_goal[4] = 0.0
        joint_goal[5] = 1.047
        joint_goal[6] = 0.785

        result = {"planner_tried": [], "plan_error_codes": [], "executed": False}
        try:
            move_group.set_joint_value_target(joint_goal)
            for planner_id in ("RRTConnect", ""):
                move_group.set_planner_id(planner_id)
                result["planner_tried"].append(planner_id or "(default)")
                plan_success, plan, planning_time, error_code = move_group.plan()
                result["plan_error_codes"].append(decode_moveit_error(error_code))
                if plan_success:
                    result["planner_used"] = planner_id or "(default)"
                    result["planning_time"] = planning_time
                    # Retime with more aggressive profile so the Franka
                    # controller receives commands of noticeable magnitude
                    # per tick -- conservative scaling compounds poorly with
                    # the 95% command acceptance rate when near a joint limit.
                    plan = move_group.retime_trajectory(
                        moveit_commander.RobotCommander().get_current_state(),
                        plan,
                        velocity_scaling_factor=0.30,
                        acceleration_scaling_factor=0.10,
                    )
                    exec_ok = move_group.execute(plan, wait=True)
                    result["executed"] = bool(exec_ok)
                    break
        finally:
            move_group.stop()
            move_group.set_max_velocity_scaling_factor(1.0)
            move_group.set_max_acceleration_scaling_factor(1.0)
        return result

    # Largest cumulative travel (rad) any single joint may make in one motion.
    # A pose goal has many IK solutions and a sampling planner happily picks
    # a far-away one (elbow flip, base spun 130 deg), which the user does not
    # want near people. 1.75 rad ~ 100 deg: moves between the working poses
    # use far less, and even home -> table pick needs only ~90 deg on the
    # shoulder; a rejected plan tells the caller to go in steps.
    MAX_JOINT_TRAVEL_RAD = 1.75

    def plan_and_execute_with_retry(self, plan_fn, auto_recover=True, max_retries=1, get_last_error=None,
                                    traj_check_margin_rad=0.02, max_joint_travel_rad=None):
        """Plan, execute, and auto-recover on execute failure.

        plan_fn: callable returning (plan, plan_ok, metadata)
          plan: RobotTrajectory
          plan_ok: bool (False if plan incomplete / planner failed)
          metadata: dict (fraction, error_code, planning_time)

        get_last_error: optional callable returning the last rosout ERROR
          string. Used to classify execute failures: external-force reflexes
          (user hand / unexpected contact) are NOT auto-recovered, because
          the user probably wants the motion to stop, not resume.

        Recovery strategy: execute failures are treated as reflex-style errors.
        On failure: call recover(), replan via plan_fn (robot state drifted),
        retry execute. Up to max_retries retries.

        Plan failures (plan_ok=False) are not retried — recover can't help a
        target that's unreachable / in collision / etc.

        max_joint_travel_rad: reject plans whose cumulative per-joint travel
          exceeds this (None -> MAX_JOINT_TRAVEL_RAD; 0 disables the check).

        Returns dict:
          outcome: 'success' | 'plan_failed' | 'execute_failed' |
                   'recovery_failed' | 'stopped' | 'stopped_external_force' |
                   'plan_would_violate_limits' | 'plan_too_large'
          recovery_triggered: bool
          recovery_succeeded: bool
          attempts: int
          plan_metadata: whatever plan_fn returned on the final attempt
          stop_reason: only present on 'stopped'/'stopped_external_force'
        """
        recovery_triggered = False
        recovery_succeeded = False
        attempts = 0
        last_meta = {}
        if max_joint_travel_rad is None:
            max_joint_travel_rad = self.MAX_JOINT_TRAVEL_RAD
        # Clear any stop flag from a previous motion so this call starts clean.
        self._stop_requested = False

        while attempts <= max_retries:
            attempts += 1
            plan, plan_ok, meta = plan_fn()
            last_meta = meta
            self.display_trajectory(plan)

            if not plan_ok:
                # Distinguish plan-failed-because-user-stop-is-pressed from
                # plan-failed-because-target-is-unreachable: the former is
                # actionable only by physically releasing the button.
                last_err = get_last_error() if get_last_error else None
                if is_user_stop_abort(last_err):
                    return {
                        'outcome': 'stopped_user_button',
                        'recovery_triggered': recovery_triggered,
                        'recovery_succeeded': recovery_succeeded,
                        'attempts': attempts,
                        'plan_metadata': last_meta,
                        'stop_reason': last_err,
                    }
                return {
                    'outcome': 'plan_failed',
                    'recovery_triggered': recovery_triggered,
                    'recovery_succeeded': recovery_succeeded,
                    'attempts': attempts,
                    'plan_metadata': last_meta,
                }

            # Trajectory-wide joint-limit check -- reject plans that would
            # drive any joint near its limit mid-path, before the firmware
            # refuses the commands.
            if traj_check_margin_rad is not None:
                traj_violations = check_trajectory_for_limits(plan, margin_rad=traj_check_margin_rad)
                if traj_violations:
                    return {
                        'outcome': 'plan_would_violate_limits',
                        'recovery_triggered': recovery_triggered,
                        'recovery_succeeded': recovery_succeeded,
                        'attempts': attempts,
                        'plan_metadata': last_meta,
                        'trajectory_violations': traj_violations,
                    }

            # Big-swing guard: refuse trajectories that wind a joint a long
            # way even though start and goal poses may be close.
            if max_joint_travel_rad:
                travel = trajectory_joint_travel(plan)
                too_far = {j: round(t, 3) for j, t in travel.items() if t > max_joint_travel_rad}
                if too_far:
                    rospy.logwarn("plan rejected: joint travel %s exceeds %.2f rad", too_far, max_joint_travel_rad)
                    return {
                        'outcome': 'plan_too_large',
                        'recovery_triggered': recovery_triggered,
                        'recovery_succeeded': recovery_succeeded,
                        'attempts': attempts,
                        'plan_metadata': last_meta,
                        'joint_travel': {j: round(t, 3) for j, t in travel.items()},
                        'max_joint_travel_rad': max_joint_travel_rad,
                    }

            ok = self.execute_plan(plan)
            # If the user called /control/stop during execution, execute_plan
            # will have returned False due to the aborted trajectory. Do NOT
            # auto-retry in that case -- the stop was intentional.
            if self._stop_requested:
                return {
                    'outcome': 'stopped',
                    'recovery_triggered': recovery_triggered,
                    'recovery_succeeded': recovery_succeeded,
                    'attempts': attempts,
                    'plan_metadata': last_meta,
                    'stop_reason': 'user /control/stop',
                }
            if ok:
                return {
                    'outcome': 'success',
                    'recovery_triggered': recovery_triggered,
                    'recovery_succeeded': recovery_succeeded,
                    'attempts': attempts,
                    'plan_metadata': last_meta,
                }

            # Execute failed. Before auto-recovering, check whether this was
            # an external-force reflex (user hand pushing the robot) or the
            # hardware user-stop button. In both cases, respect the user's
            # intent and do not retry.
            last_err = get_last_error() if get_last_error else None
            if is_user_stop_abort(last_err):
                return {
                    'outcome': 'stopped_user_button',
                    'recovery_triggered': recovery_triggered,
                    'recovery_succeeded': recovery_succeeded,
                    'attempts': attempts,
                    'plan_metadata': last_meta,
                    'stop_reason': last_err,
                }
            if is_external_force_abort(last_err):
                return {
                    'outcome': 'stopped_external_force',
                    'recovery_triggered': recovery_triggered,
                    'recovery_succeeded': recovery_succeeded,
                    'attempts': attempts,
                    'plan_metadata': last_meta,
                    'stop_reason': last_err,
                }

            if not auto_recover or attempts > max_retries:
                return {
                    'outcome': 'execute_failed',
                    'recovery_triggered': recovery_triggered,
                    'recovery_succeeded': recovery_succeeded,
                    'attempts': attempts,
                    'plan_metadata': last_meta,
                }

            # Execute failed, try recovery before retrying
            recovery_triggered = True
            recovery_succeeded = self.recover()
            if not recovery_succeeded:
                return {
                    'outcome': 'recovery_failed',
                    'recovery_triggered': True,
                    'recovery_succeeded': False,
                    'attempts': attempts,
                    'plan_metadata': last_meta,
                }

        return {
            'outcome': 'execute_failed',
            'recovery_triggered': recovery_triggered,
            'recovery_succeeded': recovery_succeeded,
            'attempts': attempts,
            'plan_metadata': last_meta,
        }

    # Weights for picking among IK solutions: swinging the whole arm round
    # (joint 1) or flipping the elbow (joint 3) is what reads as a "big
    # motion" to bystanders; wrist roll (7) barely matters.
    IK_JOINT_WEIGHTS = (2.0, 1.0, 2.0, 1.0, 1.0, 1.0, 0.5)

    def _ik_near_current(self, pose, timeout=0.2):
        """Joint values reaching ``pose`` (base frame) that stay closest to
        the current configuration. /compute_ik (collision-aware) is run from
        several seeds -- the current joints, plus "natural" configurations
        with the base joint pointed at the target and the redundant joints
        (3, 5) zeroed, since seeding from the current state alone still
        happily returns a base-spun elbow-swung solution -- and the solution
        with the smallest max joint delta from the current joints wins.
        Returns a 7-list or None if every seed fails."""
        import math as _math
        from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
        move_group = self.move_group
        try:
            rospy.wait_for_service("/compute_ik", timeout=2.0)
            ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)
            names = move_group.get_active_joints()
            cur_state = self.robot.get_current_state()
            cur_lookup = dict(zip(cur_state.joint_state.name, cur_state.joint_state.position))
            current = [cur_lookup[n] for n in names]

            j1_target = _math.atan2(pose.position.y, pose.position.x)
            seeds = [current]
            natural = list(current)
            natural[0], natural[2], natural[4] = j1_target, 0.0, 0.0
            seeds.append(natural)
            # canonical elbow-up "ready" arm aimed at the target
            seeds.append([j1_target, -0.785, 0.0, -1.571, 0.0, 1.047, 0.785])
            # same, wrist rotated the other way round (yaw ambiguity)
            seeds.append([j1_target, -0.785, 0.0, -1.571, 0.0, 1.047, 0.785 - _math.pi])

            best, best_cost = None, float("inf")
            for seed in seeds:
                req = GetPositionIKRequest()
                req.ik_request.group_name = move_group.get_name()
                st = copy.deepcopy(cur_state)
                pos = list(st.joint_state.position)
                for n, v in zip(names, seed):
                    pos[list(st.joint_state.name).index(n)] = v
                st.joint_state.position = pos
                req.ik_request.robot_state = st
                req.ik_request.ik_link_name = move_group.get_end_effector_link()
                req.ik_request.pose_stamped.header.frame_id = move_group.get_planning_frame()
                req.ik_request.pose_stamped.pose = pose
                req.ik_request.avoid_collisions = True
                req.ik_request.timeout = rospy.Duration(timeout)
                res = ik(req)
                if res.error_code.val != MoveItErrorCodes.SUCCESS:
                    continue
                lookup = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                sol = [lookup[n] for n in names]
                cost = max(w * abs(a - b) for w, a, b in zip(self.IK_JOINT_WEIGHTS, sol, current))
                if cost < best_cost:
                    best, best_cost = sol, cost
            if best is None:
                rospy.logwarn("seeded IK failed from every seed; falling back to pose goal")
                return None
            rospy.loginfo("seeded IK: max joint delta %.2f rad", best_cost)
            return best
        except Exception as e:
            rospy.logwarn("seeded IK unavailable (%s); falling back to pose goal", e)
            return None

    def _eef_orientation_constraint(self, qx=1.0, qy=0.0, qz=0.0, qw=0.0, tolerance=0.8):
        # Keep the end-effector aligned with the given target orientation
        # throughout the path. Blocks the planner from picking IK solutions that
        # flip the wrist/elbow mid-motion. tolerance is in radians on each axis.
        oc = moveit_msgs.msg.OrientationConstraint()
        oc.link_name = self.move_group.get_end_effector_link()
        oc.header.frame_id = self.move_group.get_planning_frame()
        oc.orientation.x = qx
        oc.orientation.y = qy
        oc.orientation.z = qz
        oc.orientation.w = qw
        oc.absolute_x_axis_tolerance = tolerance
        oc.absolute_y_axis_tolerance = tolerance
        oc.absolute_z_axis_tolerance = tolerance
        oc.weight = 1.0
        constraints = moveit_msgs.msg.Constraints()
        constraints.orientation_constraints.append(oc)
        return constraints

    def plan_cartesian_path(self, x, y, z=0.2, scale=1, preserve_orientation=True,
                            orientation=None):
        # orientation: optional (qx, qy, qz, qw) TCP target orientation in the
        # base frame -- takes precedence over preserve_orientation. Otherwise:
        # preserve_orientation=True carries the current wrist orientation
        # through the move; False resets to (1,0,0,0), the legacy target.
        move_group = self.move_group

        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x = scale * x
        wpose.position.y = scale * y
        wpose.position.z = scale * z
        if orientation is not None:
            (wpose.orientation.x, wpose.orientation.y,
             wpose.orientation.z, wpose.orientation.w) = orientation
        elif not preserve_orientation:
            wpose.orientation.x = 1.0
            wpose.orientation.y = 0.0
            wpose.orientation.z = 0.0
            wpose.orientation.w = 0.0
        waypoints.append(copy.deepcopy(wpose))

        # Apply orientation path constraint only when preserving orientation.
        # If the caller is intentionally changing orientation (explicit target
        # or canonical reset), constraining the path to the new orientation
        # makes the start state infeasible.
        if preserve_orientation and orientation is None:
            move_group.set_path_constraints(self._eef_orientation_constraint(
                wpose.orientation.x, wpose.orientation.y,
                wpose.orientation.z, wpose.orientation.w,
            ))
        try:
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints, 0.01
            )
        finally:
            move_group.clear_path_constraints()

        #set speed
        velocity_scaling_factor = 0.05
        plan = move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(),
                                       plan,
                                       velocity_scaling_factor)
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def plan_joint_path(self, x, y, z=0.2, scale=1, preserve_orientation=True, planner_id="LIN",
                        orientation=None):
        # orientation: optional (qx, qy, qz, qw) TCP target orientation in the
        # base frame -- takes precedence over preserve_orientation. Otherwise:
        # preserve_orientation=True carries the current wrist orientation
        # through the move; False resets to (1,0,0,0), the legacy target.
        #
        # planner_id:
        #   "LIN" (default): Pilz straight-line-in-cartesian. Good for precise
        #        short moves; prone to joint-limit drift on large workspace sweeps.
        #   "RRTConnect": OMPL joint-space sampling. Path is not
        #        straight in cartesian space, but plans are naturally limit-aware.
        #        Use for large motions where straight-line isn't required.
        move_group = self.move_group
        move_group.set_planner_id(planner_id)

        wpose = move_group.get_current_pose().pose
        wpose.position.x = scale * x
        wpose.position.y = scale * y
        wpose.position.z = scale * z
        if orientation is not None:
            (wpose.orientation.x, wpose.orientation.y,
             wpose.orientation.z, wpose.orientation.w) = orientation
        elif not preserve_orientation:
            wpose.orientation.x = 1.0
            wpose.orientation.y = 0.0
            wpose.orientation.z = 0.0
            wpose.orientation.w = 0.0
        # A pose goal lets the sampling planner pick ANY IK solution, and the
        # far ones (elbow flipped, base spun round) produce the big swings the
        # user does not want. So for the sampling planners, first solve IK
        # seeded from the current joints (KDL converges to the nearby
        # solution) and plan to that joint goal instead; fall back to the pose
        # goal only if seeded IK fails. Pilz LIN is a straight line anyway.
        joint_goal = None
        if planner_id != "LIN":
            joint_goal = self._ik_near_current(wpose)
        if joint_goal is not None:
            move_group.set_joint_value_target(joint_goal)
        else:
            move_group.set_pose_target(wpose)
        move_group.set_num_planning_attempts(10)
        move_group.set_planning_time(5.0)
        # Apply orientation constraint only when preserving orientation -- see
        # note in plan_cartesian_path for the rationale.
        if preserve_orientation and orientation is None:
            move_group.set_path_constraints(self._eef_orientation_constraint(
                wpose.orientation.x, wpose.orientation.y,
                wpose.orientation.z, wpose.orientation.w,
            ))
        try:
            # Sampling planners are stochastic: plan a few times and keep the
            # trajectory with the least joint travel.
            n_plans = 1 if planner_id == "LIN" else 3
            best = None
            for _ in range(n_plans):
                (plan_success, plan, planning_time, error_code) = move_group.plan()
                if not plan_success:
                    if best is None:
                        best = (plan_success, plan, planning_time, error_code, float("inf"))
                    continue
                cost = max(trajectory_joint_travel(plan).values() or [0.0])
                if best is None or cost < best[4]:
                    best = (plan_success, plan, planning_time, error_code, cost)
            (plan_success, plan, planning_time, error_code, _cost) = best
        finally:
            move_group.clear_path_constraints()
            move_group.clear_pose_targets()
        print(plan_success, planning_time, error_code, "joint_goal_seeded=%s" % (joint_goal is not None))
        #set speed
        velocity_scaling_factor = 0.30
        plan = move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), 
                                       plan, 
                                       velocity_scaling_factor,
                                       acceleration_scaling_factor=0.050)
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return (plan_success, plan, planning_time, error_code)

        ## END_SUB_TUTORIAL

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

        ## END_SUB_TUTORIAL

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
        success = move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL
        return success

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
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
        box_pose.header.frame_id = "panda_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
    # Real ceiling height above the robot base (panda_link0), metres. Measured
    # 2026-09-16 from the pose the arm was in when it bumped the ceiling
    # (fingertips at z=1.13). The virtual slab starts CEILING_MARGIN below it.
    VIRTUAL_CEILING_Z = 1.13
    VIRTUAL_CEILING_MARGIN = 0.05

    def add_virtual_walls(self, ceiling_z=None):
        """Two virtual walls plus a ceiling fencing the arm into the working
        region (x > -0.30, y > -0.45, z < ceiling): people stand in the other
        quadrants and there is a low ceiling above the base. With these in the
        planning scene MoveIt refuses any plan that would sweep a link into them.
        The y wall sits at -0.45, which keeps the legacy CAMERA photo pose at
        y=-0.27 reachable (user decision, 2026-09-15). The x wall was pulled in
        to -0.30 (2026-09-16) so the wrist can no longer swing back over the base
        (the pose that hit the ceiling had link6/7 at x=-0.19..-0.24); the home
        pose's elbow at x=-0.22 is still clear (verified with
        /check_state_validity). The ceiling
        slab's underside is VIRTUAL_CEILING_MARGIN below the real ceiling
        (``ceiling_z``, default VIRTUAL_CEILING_Z). All are colored transparent
        red in RViz."""
        if ceiling_z is None:
            ceiling_z = self.VIRTUAL_CEILING_Z
        import time as _time
        from moveit_msgs.msg import PlanningScene, ObjectColor
        from std_msgs.msg import ColorRGBA

        scene = self.scene
        frame = self.move_group.get_planning_frame()

        def _box(name, cx, cy, cz, sx, sy, sz):
            pose = geometry_msgs.msg.PoseStamped()
            pose.header.frame_id = frame
            pose.pose.orientation.w = 1.0
            pose.pose.position.x = cx
            pose.pose.position.y = cy
            pose.pose.position.z = cz
            scene.add_box(name, pose, size=(sx, sy, sz))

        def _wall(name, cx, cy, sx, sy):
            _box(name, cx, cy, 0.7, sx, sy, 1.4)   # spans z 0..1.4

        # Thick slab whose underside sits at ceiling_z - margin; the arm can
        # never reach above ~1.2 m anyway, so 0.5 m thickness covers it.
        ceil_sz = 0.5
        ceil_bottom = ceiling_z - self.VIRTUAL_CEILING_MARGIN

        # The PSI publisher is asynchronous and silently drops messages until
        # it is connected to move_group (bites right after a server restart),
        # so add-and-verify with retries against the reliable scene service.
        wanted = {"virtual_wall_y", "virtual_wall_x", "virtual_ceiling"}
        for _attempt in range(6):
            _wall("virtual_wall_y", 0.0, -0.45, 2.0, 0.02)   # blocks y < -0.45
            _wall("virtual_wall_x", -0.30, 0.0, 0.02, 2.0)   # blocks x < -0.30
            _box("virtual_ceiling", 0.0, 0.0, ceil_bottom + ceil_sz / 2.0,
                 2.0, 2.0, ceil_sz)                          # blocks z > ceil_bottom
            _time.sleep(0.5)
            try:
                if wanted.issubset(set(scene.get_known_object_names())):
                    break
            except Exception:
                pass
        else:
            rospy.logerr("virtual walls NOT confirmed in the planning scene!")

        # Transparent red rendering in RViz (collision behavior is unaffected
        # by color). Publish an ObjectColor diff on /planning_scene.
        try:
            pub = rospy.Publisher("/planning_scene", PlanningScene, queue_size=2)
            _time.sleep(0.5)  # let the publisher connect to move_group
            ps = PlanningScene()
            ps.is_diff = True
            for name in wanted:
                oc = ObjectColor()
                oc.id = name
                oc.color = ColorRGBA(r=1.0, g=0.25, b=0.2, a=0.35)
                ps.object_colors.append(oc)
            pub.publish(ps)
        except Exception as e:
            rospy.logwarn("virtual wall coloring failed (walls still active): %s", e)

    def add_floor(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group
        box_name = "floor"
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Floor to the Planning Scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = move_group.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = -0.1  # under the robot
        scene.add_box(box_name, box_pose, size=(2.0, 2.0, 0.2))
        # box_name = "camerarod"
        # ## BEGIN_SUB_TUTORIAL add_box
        # ##
        # ## Adding camera rod to the Planning Scene
        # box_pose = geometry_msgs.msg.PoseStamped()
        # box_pose.header.frame_id = move_group.get_planning_frame()
        # box_pose.pose.position.x = -0.5  # 
        # box_pose.pose.position.y = 0.43  # 
        # box_pose.pose.position.z = 0.315  # 
        # scene.add_box(box_name, box_pose, size=(0.1, 0.02, 0.02))
        # box_name = "kiting box"
        # box_pose1 = geometry_msgs.msg.PoseStamped()
        # box_pose1.header.frame_id = move_group.get_planning_frame()
        # box_pose1.pose.position.x = 0.488  #
        # box_pose1.pose.position.y = 0.2878  #
        # box_pose1.pose.position.z = 0.07  #
        # scene.add_box(box_name, box_pose1, size=(0.30, 0.20, 0.17))

        # box_name = "wall"
        # box_pose1 = geometry_msgs.msg.PoseStamped()
        # box_pose1.header.frame_id = move_group.get_planning_frame()
        # box_pose1.pose.position.x = 0.2  # 
        # box_pose1.pose.position.y = 0.0  # 
        # box_pose1.pose.position.z = 0.0  # 
        # scene.add_box(box_name, box_pose1, size=(0.01, 1.0, 1.0))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

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
        ## robot, we set ``grasping_group = 'panda_hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

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
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

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

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )
    def stop(self):
        # Signal any in-flight plan_and_execute_with_retry loop to treat the
        # aborted execute as user-initiated and skip auto-retry.
        self._stop_requested = True
        move_group = self.move_group
        move_group.stop()
        move_group.clear_pose_targets()
        return True

def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        tutorial.go_to_joint_state()

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        tutorial.go_to_pose_goal()

        input("============ Press `Enter` to plan and display a Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        input(
            "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        )
        tutorial.display_trajectory(cartesian_plan)

        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)

        input("============ Press `Enter` to add a box to the planning scene ...")
        tutorial.add_box()

        input("============ Press `Enter` to attach a Box to the Panda robot ...")
        tutorial.attach_box()

        input(
            "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        )
        cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        tutorial.execute_plan(cartesian_plan)

        input("============ Press `Enter` to detach the box from the Panda robot ...")
        tutorial.detach_box()

        input(
            "============ Press `Enter` to remove the box from the planning scene ..."
        )
        tutorial.remove_box()

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return