""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    JointConstraint, PlanningOptions, BoundingVolume
)
from shape_msgs.msg import SolidPrimitive
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg

# ************************************************
#           IMPORTANT DETAILS BELOW             #
# ************************************************
"""


--- TBD --- indicates the section to fill out   
... indicates the specific place to type the code
There are 4 TBD sections

Pick and place: Pick and place refers to the robotic task of grasping an object from one location (pick) and moving 
it to another location (place). 

Sequence:
1.	Initialize and 'Home' the robot to a known starting joint configuration (home position).
2.	Approach pre-grasp pose to align without disturbing it.
3.	Lower the arm and close the gripper to grasp the object.
4.	Retreat back by retreating along the vertical axis to avoid collisions with surroundings.
5.	Approach 'Place' pose above the destination location in preparation for placement.
6.	Place (Release)	Lower the object to the target surface and open the gripper to release it.
7.	Retreat back the arm back to avoid disturbing the placed object or colliding with the workspace.
8.	Reset / Home Again

"""
class SequentialPosePlanner(Node):
    def __init__(self):
        """Initializes the node, action clients, and sends the robot to the home position."""
        super().__init__('sequential_pose_planner')

        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.client.wait_for_server()

        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.gripper_client.wait_for_server()

        self.states_and_poses = []
        self.current_state_index = 0
        self.is_homed = False
        self.returned_home = False
        self.awaiting_home_result = False
        self.gripper_close_pos = -0.002
        self.gripper_open_pos = 0.01

        self.go_to_home()

        self.create_timer(2.0, self.post_home_callback)

    def quaternion_to_euler(self, q):
        """Converts a quaternion to Euler angles."""
        return euler_from_quaternion([q.x, q.y, q.z, q.w])

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Converts Euler angles to a quaternion."""
        q_array = quaternion_from_euler(roll, pitch, yaw)
        q = Quaternion()
        q.x, q.y, q.z, q.w = q_array
        return q

    def post_home_callback(self):
        """Triggered after reaching home; sets up pick-and-place tasks and begins execution."""
        #-------------------- TBD ---------------------------
        # One sequence of action to move a block from table 1 to 2 and table 2 to 1 is provided below
        # Add more sequences below to solve the puzzle
        # Refer to the figure of the manipulation base in the question for the measurements
        # the goal is to move blocks between table 1 and table 2 to rearrange the puzzle.

        if self.is_homed and not self.states_and_poses:
            # ----------------- one sample sequence is provided here ---------------
            # this picks the cube from position 1 table 1 to position 1 table 2 cw rotation
            self.add_pick_place_sequence((0.08, 0.0), (0.0, -0.08), -1)
            # --------- add more here ---------------------------
        # ------------------------- TBD -END ---------------------------------

            self.plan_to_position_goal_sequential()

    def get_pose_stamped(self, x, y, z, roll, pitch, yaw):
        """Generates a PoseStamped message from position and orientation (Euler angles)."""
        q = self.euler_to_quaternion(roll, pitch, yaw)
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = q
        return pose

    def add_pick_place_sequence(self, pick_xy, place_xy, dir=-1.0):
        """Appends a full pick-and-place sequence (pre-grasp, grasp, retreat for both) to the state list."""
        px, py = pick_xy
        qx, qy = place_xy
        sign = dir
        # ------------------ TBD ------------------------
        # Table one is directly aligned with the robot's x-axis
        # in ROS and robotics in general, the clockwise rotation = -ve angles and ccw = +ve
        # Table 2 aligns with the robot's negative y-axis
        # populate the array below that has the orientation of the three blocks to match its position in the two tables
        # the 'dir' parameter is useful to invert the sign of the rotation that might be required for table 2 to table 1
        # movement

        pick_orientation = (..., ...,  ...)   # notice the orientation of the robot, table1 and movable axes of the robot
        place_orientation = (..., ..., sign * ...) # notice the orientation of the robot, table2 and movable axes of the robot

        self.states_and_poses.extend([
            ("PICK_PRE_GRASP", self.get_pose_stamped(px, py, 0.075, *pick_orientation)),
            ("PICK_GRASP", self.get_pose_stamped(px, py, 0.03, *pick_orientation)),
            ("PICK_RETREAT", self.get_pose_stamped(px, py, 0.075, *pick_orientation)),

            ("PLACE_PRE_GRASP", self.get_pose_stamped(qx, qy, 0.075, *place_orientation)),
            ("PLACE_GRASP", self.get_pose_stamped(qx, qy, 0.04, *place_orientation)),
            ("PLACE_RETREAT", self.get_pose_stamped(qx, qy, 0.075, *place_orientation))
        ])

        # ------------------ TBD END ------------------------

        self.get_logger().info("Added pick and place sequence.")

    def set_constraints(self, pose):
        """Creates position and orientation constraints around the given pose."""
        """
        The box constraint in the set_constraints() method is a way to define a tight 3D region in which
        the end-effector must be positioned for a successful motion plan
        
        if box.dimensions = [0.01, 0.01, 0.01]
        
        This creates a 1 cm³ volume around the desired pose.
        The robot's end-effector must be within this box at the end of the motion.
        
        This ensures:
        The planner generates a precise end pose.
        Small positioning errors are tolerated (1 cm leeway), helping the planner avoid failures due to overly strict 
        requirements.
        """
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]

        region = BoundingVolume()
        region.primitives.append(box)
        region.primitive_poses.append(pose.pose)

        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = 'end_effector_link'
        pos_constraint.constraint_region = region
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose.header
        ori_constraint.link_name = 'end_effector_link'
        ori_constraint.orientation = pose.pose.orientation
        # ---------------------- TBD --------------------
        # add tolerances between 0.01 and 2pi value. This is for orientation
        # the tolerance should be based on the kinematics of the robot
        # i.e. in which axes the robot can rotate? use low values for axes that matter
        # and allow higher values for axes does not matter. low = 0.01, high > pi

        ori_constraint.absolute_x_axis_tolerance = ...
        ori_constraint.absolute_y_axis_tolerance = ...
        ori_constraint.absolute_z_axis_tolerance = ...
        ori_constraint.weight = 1.0

        # ------------------ TBD END ------------------------

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)

        return constraints

    def plan_to_position_goal_sequential(self):
        """Begins execution of the pick-and-place sequence from the first goal."""
        self.current_state_index = 0
        self.send_next_state_goal()

    def send_next_state_goal(self):
        """Sends the next pose goal to MoveIt and handles transitions and timing."""
        self.waiting_for_result = True
        self.goal_start_time = self.get_clock().now()

        if self.current_state_index >= len(self.states_and_poses):
            if not self.returned_home:
                self.get_logger().info("All states completed. Returning home...")
                self.returned_home = True
                self.awaiting_home_result = True
                self.go_to_home()
            return

        state_name, pose = self.states_and_poses[self.current_state_index]
        self.get_logger().info(
            f"[{state_name}] Sending goal {self.current_state_index + 1}/{len(self.states_and_poses)}...")
        constraints = self.set_constraints(pose)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 5
        goal_msg.planning_options.planning_scene_diff.is_diff = True

        self.client.send_goal_async(goal_msg).add_done_callback(self.pose_goal_response_callback)

    def pose_goal_response_callback(self, future):
        """Handles the response from MoveIt after sending a pose goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        goal_handle.get_result_async().add_done_callback(self.pose_result_callback)

    def pose_result_callback(self, future):
        """Processes the result of the MoveIt goal and triggers gripper actions and next state."""
        if self.current_state_index < len(self.states_and_poses):
            state_name, _ = self.states_and_poses[self.current_state_index]
        else:
            state_name = "HOME"

        if state_name == "PICK_GRASP":
            self.get_logger().info("Grasping object...")
            self.control_gripper(self.gripper_close_pos)
        elif state_name == "PLACE_GRASP":
            self.get_logger().info("Releasing object...")
            self.control_gripper(self.gripper_open_pos)

        self.waiting_for_result = False
        try:
            result = future.result().result
            if result.error_code.val == 1:
                self.get_logger().info(f"[{state_name}] Pose executed successfully.")
            else:
                self.get_logger().warn(f"[{state_name}] Planning failed with code: {result.error_code.val}")
        except Exception as e:
            self.get_logger().error(f"[{state_name}] Failed to get result: {e}")

        if self.awaiting_home_result:
            self.get_logger().info("Returned to home. Motion sequence complete.")
            self.awaiting_home_result = False
            return

        self.current_state_index += 1
        self.get_logger().info("Waiting before sending next state goal...")
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(2.0, self.trigger_next_goal_once)

    def trigger_next_goal_once(self):
        """Triggers the next goal after a timeout or successful completion."""
        if not self.waiting_for_result:
            self.send_next_state_goal()
            self.timer.cancel()
        else:
            elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
            if elapsed > 10.0:
                self.get_logger().warn("Goal timeout exceeded. Proceeding to next state...")
                self.waiting_for_result = False
                self.current_state_index += 1
                self.send_next_state_goal()
                self.timer.cancel()

    def go_to_home(self):
        """Sends the robot arm to the predefined home joint configuration."""
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        # --------------------- TBD --------------------
        # A common practice in ROS is to define preset positions for the robot arm that are frequently used
        # for instance, the robot's home position  or the start joint angles.
        #  This is stored in a SRDF file undet he package: open_manipulator_x_moveit_config/config
        # Populate the joint_values array below using the values from the SRDF file

        joint_values = [..., ..., ..., ...]

        # --------------------- TBD - END --------------------

        joint_constraints = []
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)

        constraints = Constraints()
        constraints.joint_constraints = joint_constraints

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 5
        goal_msg.planning_options.planning_scene_diff.is_diff = True

        self.get_logger().info('Sending home joint goal...')
        self.client.send_goal_async(goal_msg).add_done_callback(self.pose_goal_response_callback)
        self.is_homed = True
        self.control_gripper(0.01)

    def control_gripper(self, position, max_effort=0.5):
        """Sends a command to the gripper to open or close to a specific position."""
        goal_msg = GripperCommand.Goal()
        command = GripperCommandMsg()
        command.position = position
        command.max_effort = max_effort
        goal_msg.command = command

        self.get_logger().info(f"Sending gripper command to position: {position}")
        self.gripper_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SequentialPosePlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
