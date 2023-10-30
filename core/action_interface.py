#!/usr/bin/env python

import os, sys
import numpy as np
import re
import datetime
import math
import copy
import rospy
import matplotlib.pyplot as plt
import message_filters
from stag_ros.msg import *
from apriltag_ros.msg import *
from geometry_msgs.msg import Twist

from panda_interface.arm import PandaArmInterface
from core.utils.general_utils import AttrDict
import core.utils.ros_transform_utils as tfu
from core.utils.transform_utils import (
    quat2mat,
    mat2quat,
    axisangle2quat,
    quat2axisangle,
    quat_multiply,
    make_pose,
    mat2pose,
)
from core.const import BEAMS, PEGS
from planner.beam_assembly.beam_assembly_parser import ElementType


class ActionInterface(object):
    def __init__(self, fixed_beam_id="b7"):
        """
        Args:
            fixed_beam_id: ID of a beam fixed on a table
        """

        rospy.init_node("action_interface")
        self.top_marker_subscriber = rospy.Subscriber(
            "top_camera/tag_detections",
            AprilTagDetectionArray,
            self._update_top_markers,
        )
        self.wrist_marker_subscriber = rospy.Subscriber(
            "wrist_camera/tag_detections",
            AprilTagDetectionArray,
            self._update_wrist_markers,
        )
        self.top_markers = AttrDict()
        self.wrist_markers = AttrDict()

        self.fixed_beam_id = fixed_beam_id

        # Panda arm interface
        self.robot = PandaArmInterface(AttrDict())
        self.rate = rospy.Rate(5)

        self.base_orientation = axisangle2quat(np.array([-np.pi, 0, 0]))
        self.beams = BEAMS
        self.pegs = PEGS

        self.grasped_beam_id = None
        self.grasped_beam_link_id = None

        # add collision boxes around a robot arm to avoid unexpected collision with a table and wall
        self.construct_scene()

    def _update_top_markers(self, top_msg):
        """
        Update marker poses observed by a top-view camera
        """
        self.top_current_markers = AttrDict()
        for tag_marker in top_msg.detections:
            marker_pose = tfu.current_robot_pose(
                "panda_link0", "tag_" + str(tag_marker.id[0])
            )
            self.top_markers["tag_{}".format(tag_marker.id[0])] = AttrDict(
                pose=marker_pose, time=datetime.datetime.now()
            )
            self.top_current_markers["tag_{}".format(tag_marker.id[0])] = marker_pose

    def _update_wrist_markers(self, wrist_msg):
        """
        Update marker poses observed by a wrist-camera
        """
        self.wrist_current_markers = AttrDict()
        for tag_marker in wrist_msg.detections:
            marker_pose = tfu.current_robot_pose(
                "panda_link0", "tag_" + str(tag_marker.id[0])
            )
            self.wrist_markers["tag_{}".format(tag_marker.id[0])] = AttrDict(
                pose=marker_pose, time=datetime.datetime.now()
            )
            self.wrist_current_markers["tag_{}".format(tag_marker.id[0])] = marker_pose

    def pickup_beam(self, target):
        """
        Grasp a beam at a specific link
        Args:
            target (str): A target beam id with link id for grasping. E.g. b7l1
        """

        beam_id, link_id = self.get_beam_link_id(target)

        link_pose = self.get_beam_link_pose(beam_id, link_id)
        pos, ori = mat2pose(link_pose)
        grasped_link_euler = quat2axisangle(ori)

        # use these information to update the approach pose later.
        self.grasped_link_euler = grasped_link_euler
        self.grasped_beam_id = beam_id
        self.grasped_beam_link_id = link_id

        # open the gripper and move above the target link
        self.robot.gripper.open()
        self.robot.move_to_cartesian_pose([pos[0], pos[1], 0.1])

        # close the gripper slightly so that the gripper does not hit to other objects
        # self.robot.gripper.move_joints(0.035)
        self.robot.gripper.move_joints(0.04)
        self.robot.move_to_cartesian_pose([pos[0], pos[1], 0.035])
        # grasp the beam
        self.robot.gripper.grasp(0.01, force=80)
        print("Grasped!")

        # move upwards
        self.robot.move_to_cartesian_delta([0, 0, 0.1])
        return True

    def move_to_input_area(self):
        """
        Move to the input area.
        """
        print("Moving to the input area")
        self.robot.move_to_cartesian_pose(
            [
                self.input_origin.position.x - 0.2,
                self.input_origin.position.y + 0.1,
                self.input_origin.position.z + 0.4,
            ]
        )

    def move_to_intermediate_area(self):
        """
        Move to the intermediate area.
        """
        print("Moving to the intermediate area")
        self.robot.move_to_cartesian_pose(
            [
                self.input_origin.position.x - 0.2,
                self.input_origin.position.y - 0.1,
                self.input_origin.position.z + 0.4,
            ]
        )
        self.move_neutral_ori()

    def move_to_assembly_area(self):
        """
        Move to the assembly area.
        """
        print("Moving to the assembly area")
        self.robot.move_to_cartesian_pose(
            [
                self.input_origin.position.x - 0.2,
                self.input_origin.position.y - 0.3,
                self.input_origin.position.z + 0.4,
            ]
        )

    def move_beam_to_approach_pose(self, target, insert_end):
        """
        Move the beam to the approach pose for insertion
        Args:
            target (str): A target beam id.
            insert_end (BeamComponent): An instance of BeamComponent class containing the insertion endpoint joint.
        """

        print("Moving to the approach pose")
        # Transform the approach pose of the insertion endpoint to the approach poes of the link that the robot grasps.
        self.update_beam_approach_pose(
            self.grasped_beam_id, self.grasped_beam_link_id, insert_end
        )
        self.move_neutral_ori()

        target_pose = self.beams[target]
        trans, quat = mat2pose(target_pose)
        euler = quat2axisangle(quat)

        self.robot.move_to_cartesian_pose([trans[0], trans[1], 0.15])

        # FIXME: rotate a gripper with the difference in orientation of grasped and target beam
        if np.abs(self.grasped_link_euler[2] - euler[2]) > 2:
            self.rotate_gripper(np.pi)
        elif np.abs(self.grasped_link_euler[2] - euler[2]) > 1.0:
            self.rotate_gripper(np.pi / 2.0)

        # Slowly move the beam to the approach pose.
        current_ori = self.robot.tip_pose().orientation
        self.robot.move_to_cartesian_delta([0, 0, -0.05])
        self.robot.move_to_cartesian_delta([0, 0, -0.05])
        self.robot.move_to_cartesian_pose([trans[0], trans[1], 0.038])

    def move_to_beam(self, target):
        """
        Move to the origin of the target beam
        Args:
            target (str): A target beam id
        """

        print("Moving to the beam {}".format(target))
        self.move_neutral_ori()
        beam_id = self.get_beam_id(target)

        success = False
        while not success:
            marker_pose = self.get_marker_pose(self.beams[beam_id].origin_id)
            success = self.robot.move_to_cartesian_pose(
                [marker_pose.position.x - 0.01, marker_pose.position.y, 0.2]
            )
        rospy.sleep(1.5)

    def move_to_peg(self, target):
        """
        Move to the target peg
        Args:
            target (str): A target peg id
        """

        print("Moving to the peg {}".format(target))
        peg_id = re.match("(p\d*)i", target)[1]
        marker_id = self.pegs[peg_id]
        success = False
        while not success:
            marker_pose = self.get_marker_pose(marker_id)
            success = self.robot.move_to_cartesian_pose(
                [
                    marker_pose.position.x,
                    marker_pose.position.y,
                    marker_pose.position.z + 0.15,
                ]
            )
        rospy.sleep(1.5)
        self.move_neutral_ori()
        return True

    def pickup_peg(self, target):
        """
        Grasp a peg
        Args:
            marker_id: the peg's marker id
        """

        print("Pick up the peg {}".format(target))
        marker_id = self.pegs[target]
        marker_pose = self.get_marker_pose(marker_id)
        self.robot.gripper.open()

        self.robot.move_to_cartesian_pose(
            [marker_pose.position.x, marker_pose.position.y, 0.14]
        )

        self.robot.move_to_cartesian_pose(
            [marker_pose.position.x+0.005, marker_pose.position.y, 0.085]
        )
        self.robot.gripper.grasp(0.01, force=100)
        rospy.sleep(0.5)
        self.robot.move_to_cartesian_delta([0, 0, 0.165])
        self.move_neutral_ori()
        return True

    def insert_peg(self, bj1, bj2, peg):
        """
        Insert the grasped peg to the hole of bj1 and bj2.
        Args:
            bj1: A beam id with a joint id
            bj2: A beam id with a joint id
            peg: A peg id
        """

        print("Inserting the peg {}...".format(peg))
        # Move closer
        beam1_id, beam1_joint_id = self.get_beam_joint_id(bj1)
        beam2_id, beam2_joint_id = self.get_beam_joint_id(bj2)
        pose1 = self.get_beam_joint_pose(beam1_id, beam1_joint_id)
        pose2 = self.get_beam_joint_pose(beam2_id, beam2_joint_id)
        pos = (mat2pose(pose1)[0] + mat2pose(pose2)[0]) / 2.0
        self.robot.move_to_cartesian_pose([pos[0] - 0.03, pos[1], 0.25])

        rospy.sleep(0.5)

        # Move closer again to observe more accurate hole pose.
        beam1_id, beam1_joint_id = self.get_beam_joint_id(bj1)
        beam2_id, beam2_joint_id = self.get_beam_joint_id(bj2)
        pose1 = self.get_beam_joint_pose(beam1_id, beam1_joint_id)
        pose2 = self.get_beam_joint_pose(beam2_id, beam2_joint_id)
        pos = (mat2pose(pose1)[0] + mat2pose(pose2)[0]) / 2.0
        # self.robot.move_to_cartesian_pose([pos[0] - 0.03, pos[1], 0.2])
        self.robot.move_to_cartesian_pose([pos[0] - 0.03, pos[1], 0.16])

        rospy.sleep(1.)

        # Observe a sequence of hole poses and average them to acquire the accurate hole pose.
        pos_list = []
        for _ in range(100):
            pose1 = self.get_beam_joint_pose(beam1_id, beam1_joint_id)
            pose2 = self.get_beam_joint_pose(beam2_id, beam2_joint_id)
            pos = (mat2pose(pose1)[0] + mat2pose(pose2)[0]) / 2.0
            rospy.sleep(0.01)
            pos_list.append(pos)
        pos = np.mean(pos_list, axis=0)
        # self.robot.move_to_cartesian_pose([pos[0] + 0.005, pos[1], 0.12])
        self.robot.move_to_cartesian_pose([pos[0], pos[1] + 0.01, 0.115])

        current_pose = self.robot.tip_pose()
        base_ori = current_pose.orientation
        inserted = False
        trials = 0
        while not inserted:
            hit_bottom = False
            while not hit_bottom:
                # Keep moving downwards until the peg hits to the surface of the beams
                desired_pos = self.robot.tip_pose().position
                desired_pos[-1] -= 0.007
                self.exec_pose_cmd(desired_pos, base_ori)
                hit_bottom = np.abs(self.robot.tip_effort().force[2]) > 4.0

            print("Hit to the surface of the beam")

            z_pos = self.robot.tip_pose().position[-1]
            max_timesteps = 120
            current_pos = self.robot.tip_pose().position
            # Apply spiral search for the peg insertion
            print("Searching the hole...")
            for t in range(max_timesteps):
                rad = math.radians(t * 4)
                x = 0.005 * math.exp(0.015 * rad) * math.cos(rad)
                y = 0.005 * math.exp(0.015 * rad) * math.sin(rad)
                desired_pos = np.array(
                    [
                        current_pos[0] + x,
                        current_pos[1] + y,
                        # self.robot.tip_pose().position[2] - 0.005,
                        self.robot.tip_pose().position[2] - 0.003,
                    ]
                )
                self.exec_pose_cmd(pos=desired_pos, ori=base_ori)
                # stop if the tip of the robot is lower than a threshold
                if (
                    z_pos - self.robot.tip_pose().position[-1] > 0.005
                    or self.robot.tip_pose().position[-1] < 0.09
                ):
                    inserted = True
                    break

            # Move upwards and observe the hole position again to repeat the insertion process
            if not inserted:
                print("Retry the insertion")
                self.robot.move_to_cartesian_pose([pos[0] - 0.02, pos[1], 0.2])
                rospy.sleep(0.5)
                for _ in range(20):
                    pose = self.get_beam_joint_pose(beam1_id, beam1_joint_id)
                    pose2 = self.get_beam_joint_pose(beam2_id, beam2_joint_id)
                    pos = (mat2pose(pose)[0] + mat2pose(pose2)[0]) / 2.0
                    rospy.sleep(0.05)
                    pos_list.append(pos)
                pos = np.mean(pos_list, axis=0)
                self.robot.move_to_cartesian_pose([pos[0] + 0.005, pos[1], 0.13])

                trials += 1
                if trials > 5:
                    break

        # Move backwards and forward to avoid from jamming.
        print("Avoid from jamming..")
        for t in range(50):
            rad = math.radians(t * 6)
            x = 0.005 * math.cos(rad)
            self.exec_pose_delta_cmd(np.array([x, 0, -0.015]))
            if (
                np.abs(self.robot.tip_effort().force[2]) < 3
                and self.robot.tip_pose().position[-1] < 0.09
            ):
                break
        self.exec_pose_delta_cmd(np.array([0, 0, 0]))
        self.robot.recover_from_errors()
        return True

    def assemble_beam_square(self, grasped_beam, target_beam_component):
        """
        Assemble beams.
        Args:
            grasped_beam (str): A grasped beam id with a joint id
            target_beam_component (BeamComponent): A grasped beam component
        """
        target_beam = target_beam_component.name
        current_pose = self.robot.tip_pose()

        grasped_beam_id, grasped_joint_id = self.get_beam_joint_id(grasped_beam)
        beam_id, joint_id = self.get_beam_joint_id(target_beam)

        if target_beam_component.type.value == ElementType.THRU_F.value:
            self.assemble_thru_f(grasped_beam, target_beam)
        elif target_beam_component.type.value == ElementType.IN_F_END.value:
            self.assemble_in_f_end(grasped_beam, target_beam)
        elif target_beam_component.type.value == ElementType.IN_F.value:
            self.assemble_in_f(grasped_beam, target_beam)

        self.exec_pose_delta_cmd(np.array([0, 0, 0]))
        rospy.sleep(0.5)
        return True

    def assemble_thru_f(self, grasped_beam, target_beam):
        """
        Assembly for thru_f joint type
        Assemble beams.
        Args:
            grasped_beam (str): A grasped beam id with a joint id
            target_beamt (str): A target beam id with a joint id
        """

        print("Assemble thru_f...")
        grasped_beam_id, grasped_joint_id = self.get_beam_joint_id(grasped_beam)
        beam_id, joint_id = self.get_beam_joint_id(target_beam)

        base_pos = self.robot.tip_pose().position
        base_ori = self.robot.tip_pose().orientation
        connect = False

        # find the insertion axis
        insert_axis = np.where(
            np.abs(
                self.beams["{}t".format(grasped_beam_id)][:3, 3]
                - self.beams["{}a".format(grasped_beam_id)][:3, 3]
            )[:2]
            > 0.02
        )[0][0]

        # get the insertion direction
        insert_sign = np.sign(
            self.beams["{}t".format(grasped_beam_id)][:3, 3][insert_axis]
            - self.beams["{}a".format(grasped_beam_id)][:3, 3][insert_axis]
        )

        # Move to the target pose with an impedance controller
        while not connect:
            current_pose = self.robot.tip_pose()
            desired_position = current_pose.position
            mask = np.ones_like(desired_position, bool)
            mask[insert_axis] = False
            desired_position[insert_axis] += insert_sign * 0.01
            desired_position[mask] = base_pos[mask]
            self.exec_pose_cmd(desired_position, ori=base_ori)
            connect = np.abs(self.robot.tip_effort().force[insert_axis]) > 8.0

        print("Hit to the other beam")

        # if the current robot pose is not at the target pose
        current_pose = self.robot.tip_pose()

        t = 0
        base_pose = self.robot.tip_pose()
        base_pos = base_pose.position
        # Search the target beam joint to connect the beams.
        print("Searching the target beam joint to connect the grasped beam...")
        while (
            current_pose.position[insert_axis]
            - self.beams["{}t".format(grasped_beam_id)][insert_axis, 3]
        ) > 0.01 or np.abs(
            self.robot.tip_effort().force[insert_axis]
        ) < 8.0:  # FIXME
            current_pose = self.robot.tip_pose()
            current_pos = current_pose.position
            rad = math.radians(t * 4)
            offset = 0.01 * math.exp(0.06 * rad) * math.sin(rad)

            desired_pos = current_pos.copy()
            desired_pos[-1] = base_pos[2]
            desired_pos[insert_axis] += insert_sign * 0.006
            mask = np.ones_like(desired_pos, bool)
            mask[insert_axis] = False
            mask[-1] = False
            desired_pos[mask] += offset
            self.exec_pose_cmd(pos=desired_pos, ori=base_ori)
            t += 1

        # Adjust the beam position to align holes.
        print("Adjusting the beam position to align holes...")
        for _ in range(2):
            current_pose = self.robot.tip_pose()
            current_pos = current_pose.position
            # fix this heuristic
            delta_insert = (
                self.beams["{}t".format(grasped_beam_id)][insert_axis, 3] - 0.01
            ) - current_pos[insert_axis]
            delta_pos = np.zeros_like(current_pos)
            delta_pos[insert_axis] = delta_insert
            self.exec_pose_delta_cmd(delta_pos)

    def assemble_in_f(self, grasped_beam, target_beam):
        """
        Assembly for in_f joint type
        Assemble beams.
        Args:
            grasped_beam (str): A grasped beam id with a joint id
            target_beamt (str): A target beam id with a joint id
        """

        print("Assemble in_f...")

        grasped_beam_id, grasped_joint_id = self.get_beam_joint_id(grasped_beam)
        beam_id, joint_id = self.get_beam_joint_id(target_beam)

        base_pos = self.robot.tip_pose().position
        base_ori = self.robot.tip_pose().orientation
        connect = False
        # find the insertion axis
        insert_axis = np.where(
            np.abs(
                self.beams["{}t".format(grasped_beam_id)][:3, 3]
                - self.beams["{}a".format(grasped_beam_id)][:3, 3]
            )[:2]
            > 0.02
        )[0][0]
        # get the insertion direction
        insert_sign = np.sign(
            self.beams["{}t".format(grasped_beam_id)][:3, 3][insert_axis]
            - self.beams["{}a".format(grasped_beam_id)][:3, 3][insert_axis]
        )

        # Move to the target pose with an impedance controller
        while not connect:
            current_pose = self.robot.tip_pose()
            desired_position = current_pose.position
            mask = np.ones_like(desired_position, bool)
            mask[insert_axis] = False
            desired_position[insert_axis] += insert_sign * 0.006
            desired_position[mask] = base_pos[mask]
            self.exec_pose_cmd(desired_position, ori=base_ori)
            connect = np.abs(self.robot.tip_effort().force[insert_axis]) > 3.5
        print("Hit to the surface of the beam")

        t = 0
        base_pose = self.robot.tip_pose()
        base_pos = base_pose.position
        side_axis = 0 if insert_axis == 1 else 1
        # Insert the beams
        print("Searching the target beam joint to connect the grasped beam...")
        while (
            np.abs(self.robot.tip_effort().force[insert_axis]) < 8.0
            or np.abs(self.robot.tip_effort().force[side_axis]) < 8.0
        ):  # FIXME
            current_pose = self.robot.tip_pose()
            current_pos = current_pose.position
            rad = math.radians(t * 10)
            offset = 0.005 * math.exp(0.04 * rad) * math.sin(rad)

            desired_pos = current_pos.copy()
            desired_pos[-1] = base_pos[2]
            if np.abs(self.robot.tip_effort().force[insert_axis]) > 5:
                desired_pos[insert_axis] += insert_sign * 0.003
            else:
                desired_pos[insert_axis] += insert_sign * 0.006
            mask = np.ones_like(desired_pos, bool)
            mask[insert_axis] = False
            mask[-1] = False
            desired_pos[mask] = base_pos[mask] + offset
            self.exec_pose_cmd(pos=desired_pos, ori=base_ori)
            t += 1

    def assemble_in_f_end(self, grasped_beam, target_beam):
        """
        Assembly for in_f_end joint type
        Assemble beams.
        Args:
            grasped_beam (str): A grasped beam id with a joint id
            target_beamt (str): A target beam id with a joint id
        """

        print("Assemble in_f_end...")
        grasped_beam_id, grasped_joint_id = self.get_beam_joint_id(grasped_beam)
        beam_id, joint_id = self.get_beam_joint_id(target_beam)

        target_beam_marker_pose = self.get_marker_pose(
            self.beams[beam_id].joint_to_marker[joint_id].marker_id
        )

        grasped_beam_marker_mat = self.beams["{}a".format(grasped_beam_id)]
        target_beam_marker_mat = make_pose(
            [
                target_beam_marker_pose.position.x,
                target_beam_marker_pose.position.y,
                target_beam_marker_pose.position.z,
            ],
            quat2mat(
                [
                    target_beam_marker_pose.orientation.x,
                    target_beam_marker_pose.orientation.y,
                    target_beam_marker_pose.orientation.z,
                    target_beam_marker_pose.orientation.w,
                ]
            ),
        )

        beam_angles = quat2axisangle(
            mat2quat(
                np.dot(
                    target_beam_marker_mat[:3, :3], grasped_beam_marker_mat[:3, :3].T
                )
            )
        )

        base_pos = self.robot.tip_pose().position
        base_ori = self.robot.tip_pose().orientation
        connect = False
        # Move to the target pose with an impedance controller

        self.robot.move_to_cartesian_delta([0, -np.sin(beam_angles[-1]) * 0.01, 0.])
        base_pos[1] -= np.sin(beam_angles[-1]*0.01)
        while not connect:
            current_pose = self.robot.tip_pose()
            desired_position = current_pose.position
            desired_position[0] -= 0.01
            self.exec_pose_cmd(
                [desired_position[0], base_pos[1], base_pos[2]], ori=base_ori
            )
            print(self.robot.tip_effort().force[0])
            connect = np.abs(self.robot.tip_effort().force[0]) > 8.0
        print("Hit to the surface of the beam")

        t = 0
        base_pose = self.robot.tip_pose()
        base_pos = base_pose.position
        # Search the target beam joint to connect the beams.
        print("Searching the target beam joint to connect the grasped beam...")
        # while (
        #     current_pose.position[:1] - self.beams["{}t".format(grasped_beam_id)][:1, 3]
        #     > 0.01
        #     or np.abs(self.robot.tip_effort().force[0]) < 8.0
        # ):  # FIXME
        while (
            current_pose.position[:1] - self.beams["{}t".format(grasped_beam_id)][:1, 3]
            > 0.01
            and np.abs(self.robot.tip_effort().force[0]) < 10.0
        ):  # FIXME
            current_pose = self.robot.tip_pose()
            current_pos = current_pose.position
            rad = math.radians(t * 4)
            # y = 0.01 * math.exp(0.06 * rad) * math.sin(rad)
            y = 0.005 * math.exp(0.06 * rad) * math.sin(rad)
            desired_pos = np.array(
                [current_pos[0] - 0.004, base_pos[1] + y, base_pos[2]]
            )
            self.exec_pose_cmd(pos=desired_pos, ori=base_ori)
            t += 1

        connect = False
        pre_F = self.robot.tip_effort().force[1]
        # Align the beams
        print("Aligning the beams...")
        while not connect:
            current_pose = self.robot.tip_pose()
            desired_position = current_pose.position
            desired_position[0] -= 0.007
            # desired_position[0] -= 0.02
            desired_position[1] += np.sin(beam_angles[-1]) * 0.007
            # desired_position[1] += np.sin(beam_angles[-1]) * 0.02
            self.exec_pose_cmd(
                np.array([desired_position[0], desired_position[1], base_pos[2]]),
                ori=base_ori,
            )
            current_pose = self.robot.tip_pose()
            self.exec_pose_cmd(current_pose.position, ori=base_ori)
            connect = np.abs(self.robot.tip_effort().force[1]) > 6.0

    def put_down(self):
        """
        Put down the grasped object
        """
        print("Put down the grasped object")
        self.exec_pose_delta_cmd(np.array([0, 0, 0.05]))
        self.exec_pose_delta_cmd(np.array([0, 0, 0.0]))
        rospy.sleep(0.5)
        self.robot.switch_controllers("position_joint_trajectory_controller")
        self.robot.recover_from_errors()
        self.robot.gripper.open()
        self.robot.move_to_cartesian_delta([0, 0, 0.05])
        self.robot.move_to_cartesian_delta([0, 0, 0.1])
        return True

    def exec_pose_cmd(self, pos, ori=None):
        """
        Execute the cartesian impedance controller
        Args:
            pos: (x, y, z) position
            ori: (x, y, z, w) quaternion
        """
        if ori is None:
            ori = self.robot.tip_pose().orientation
        self.robot.exec_cartesian_pose_impedance_cmd(pos, ori=ori)
        self.rate.sleep()

    def exec_pose_delta_cmd(self, pos, ori=None):
        """
        Execute the cartesian impedance controller taking delta pose and orientation from the current pose as input.
        Args:
            pos: (x, y, z) delta position
            ori: (x, y, z, w) delta quaternion
        """
        current_pose = self.robot.tip_pose()
        desired_pos = current_pose.position + pos
        if ori is None:
            desired_ori = self.robot.tip_pose().orientation
        else:
            desired_ori = quat_multiply(current_pose.orientation, ori)
        self.robot.exec_cartesian_pose_impedance_cmd(desired_pos, ori=desired_ori)
        self.rate.sleep()

    def move_neutral_ori(self):
        """
        Rotate the gripper back to the base orientation.
        """
        current_pose = self.robot.tip_pose()
        self.robot.move_to_cartesian_pose(
            current_pose.position, ori=self.base_orientation
        )

    def rotate_gripper(self, z_angle):
        """
        Rotate the gripper
        Args:
            z_angle: (ax) axis-angle of the z-coordinate
        """
        current_pose = self.robot.tip_pose()
        current_pos = current_pose.position
        current_ori = current_pose.orientation
        desired_angles = quat2axisangle(current_ori)
        desired_angles[-1] += z_angle

        desired_angles[-1] = np.clip(desired_angles[-1], -2.0, 2.0)
        # self.robot.move_to_cartesian_pose(current_pos,
        #                                   ori=desired_ori)
        self.robot.move_to_cartesian_delta(
            np.zeros(3), ori=axisangle2quat(np.array([0, 0, z_angle]))
        )

    def cap_beams(self, grasped_beam, target_beam):
        """
        Capping beams
        Args:
            grasped_beam (str): A grasped beam id with a joint id
            target_beam (str): A target beam id with a joint id
        """

        print("Cap the beams")
        base_ori = self.robot.tip_pose().orientation
        grasped_beam_id, grasped_joint_id = self.get_beam_joint_id(grasped_beam)
        target_beam_id, target_joint_id = self.get_beam_joint_id(target_beam)

        grasped_beam_pos, grasped_beam_quat = mat2pose(
            self.get_beam_joint_pose(grasped_beam_id, grasped_joint_id)
        )
        target_beam_pos, target_beam_quat = mat2pose(
            self.get_beam_joint_pose(target_beam_id, target_joint_id)
        )

        grasped_beam_z_angle = quat2axisangle(grasped_beam_quat)[-1]
        target_beam_z_angle = quat2axisangle(target_beam_quat)[-1]

        # self.robot.move_to_cartesian_delta([0, -0.005, 0])
        # self.robot.move_to_cartesian_delta([0, -0.008, 0])
        self.robot.move_to_cartesian_delta([-0.015, 0, 0])

        rospy.sleep(0.5)
        connect = False
        pre_F = self.robot.tip_effort().force[0]
        base_pos = self.robot.tip_pose().position
        t = 0
        # capping beams using a cartesian impedance controller
        print("Capping the beams...")
        while not connect:
            desired_pos = self.robot.tip_pose().position
            self.exec_pose_cmd(
                [desired_pos[0] - 0.01, base_pos[1], base_pos[2] - 0.005], ori=base_ori
            )
            desired_pos = self.robot.tip_pose().position
            self.exec_pose_cmd([desired_pos[0], base_pos[1], base_pos[2]], ori=base_ori)
            target_beam_pos, target_beam_quat = mat2pose(
                self.get_beam_joint_pose(target_beam_id, target_joint_id)
            )
            connect = (
                np.abs(self.robot.tip_effort().force[0]) > 8.0
                and np.linalg.norm(
                    target_beam_pos[0] - self.robot.tip_pose().position[0]
                )
                < 0.003
            )
            print(np.linalg.norm(
                    target_beam_pos[0] - self.robot.tip_pose().position[0]
                ))
            t += 1
            if t > 50:
                break

        return True

    def push(self, beam_id, connected_beam_joint, pre_cap=False):
        """
        Capping a square
        Args:
            beam_id: the first beam whose position needs to be adjusted
            connected_beam_joint: A beam id with a joint id connected to the pushed beam
        """

        print("Push the beam {}".format(beam_id))
        pos, ori = mat2pose(self.get_beam_joint_pose(beam_id, "j1"))  # fix this later

        self.robot.move_to_cartesian_pose([pos[0] - 0.005, pos[1], pos[2] + 0.2])
        rospy.sleep(0.5)

        pos, ori = mat2pose(self.get_beam_link_pose(beam_id, "l1"))  # fix this later

        self.robot.move_to_cartesian_pose([pos[0], pos[1], pos[2] + 0.2])

        pos, ori = mat2pose(self.get_beam_link_pose(beam_id, "l1"))  # fix this later

        self.robot.move_to_cartesian_pose([pos[0], pos[1], 0.044])

        connected_beam_id, connected_joint_id = self.get_beam_joint_id(
            connected_beam_joint
        )
        origin_pos, origin_ori = mat2pose(self.get_beam_origin_pose(connected_beam_id))

        direction = np.cos(quat2axisangle(origin_ori)[2])
        if "j1" not in connected_joint_id:
            direction *= -1

        # if the beam is already capped, push the beam to the other direction
        if pre_cap:
            direction *= -1
        base_pose = self.robot.tip_pose()
        base_pos = base_pose.position
        base_ori = base_pose.orientation
        pushed = False

        self.robot.recover_from_errors()

        # pushing a beam
        for _ in range(100):
            current_pose = self.robot.tip_pose()
            desired_position = current_pose.position
            # desired_position[1] += direction * 0.01
            desired_position[1] += direction * 0.008
            self.exec_pose_cmd(
                np.array([base_pos[0], desired_position[1], base_pos[2]]), ori=base_ori
            )
            pushed = np.abs(self.robot.tip_effort().force[1]) > 4

            # if the beam is not capped yet, the robot should not push the beam too much
            if not pre_cap:
                if np.abs(self.robot.tip_pose().position[1] - base_pos[1]) > 0.02:
                    pushed = True

            if pushed:
                break

        self.robot.recover_from_errors()
        self.robot.move_to_cartesian_delta(np.array([0.0, -direction * 0.01, 0]))
        self.robot.move_to_cartesian_delta(np.array([0.0, 0.0, 0.2]))
        self.move_neutral_ori()

    def get_beam_origin_pose(self, beam_id):
        """
        Get the origin pose of the given beam id
        Args:
            beam_id (str): A beam id
        """
        beam_info = self.beams[beam_id]
        origin_id = beam_info.origin_id
        marker_pose = self.get_marker_pose(origin_id)
        pos = marker_pose.position
        ori = marker_pose.orientation
        rot_mat = quat2mat([ori.x, ori.y, ori.z, ori.w])
        pose_mat = make_pose([pos.x, pos.y, pos.z], rot_mat)
        local_pose_mat = make_pose(beam_info.offset, np.eye(3))
        origin_pose = np.dot(pose_mat, local_pose_mat)

        return origin_pose

    def get_beam_link_pose(self, beam_id, link_id):
        """
        Get the origin pose of the given beam id
        Args:
            beam_id (str): A beam id
            link_id (str): A link id
        """

        origin_pose = self.get_beam_origin_pose(beam_id)
        beam_info = self.beams[beam_id]
        local_pose_mat = make_pose(beam_info.link_offsets[link_id], np.eye(3))
        link_pose = np.dot(origin_pose, local_pose_mat)
        return link_pose

    def get_beam_joint_pose(self, beam_id, joint_id=None):
        """
        Calculate the joint poes of the beam
        Args:
            beam_id (str): A beam id with or without a joint id
            joint_id (str): A joint id. Optional if the beam_id contains the joint id
        """

        if joint_id is None:
            beam_id, joint_id = self.get_beam_joint_id(beam_id)

        if joint_id not in self.beams[beam_id].joint_to_marker.keys():
            origin_pose = self.get_beam_origin_pose(beam_id)
            beam_info = self.beams[beam_id]
            local_pose_mat = make_pose(beam_info.joint_offsets[joint_id], np.eye(3))
            joint_pose = np.dot(origin_pose, local_pose_mat)
        else:
            joint_info = self.beams[beam_id].joint_to_marker[joint_id]
            marker_pose = self.get_marker_pose(joint_info.marker_id)
            pos = marker_pose.position
            ori = marker_pose.orientation
            rot_mat = quat2mat([ori.x, ori.y, ori.z, ori.w])
            pose_mat = make_pose([pos.x, pos.y, pos.z], rot_mat)
            local_pose_mat = make_pose(joint_info.offset, np.eye(3))
            joint_pose = np.dot(pose_mat, local_pose_mat)

        return joint_pose

    def get_beam_joint_id(self, beam):
        """
        get the beam and joint id
        args:
            beam (str): a beam id with a joint id
        """
        group = re.match("(b\d*)(j\d.*)", beam)
        return group[1], group[2]

    def get_beam_link_id(self, beam):
        """
        get the beam and link id
        args:
            beam (str): a beam id with a link id
        """
        group = re.match("(b\d*)(l\d.*)", beam)
        return group[1], group[2]

    def get_beam_id(self, beam):
        """
        get the beam id
        args:
            beam (str): A beam id with a joint or link id
        """
        group = re.match("(b\d*)(.*)", beam)
        beam_id = group[1]
        return beam_id

    def record_fixed_origin(self):
        """
        Record the fixed beam origin pose and the pose of the marker_id 0.
        """
        rospy.sleep(1.0)
        self.fixed_beam_origin = self.get_beam_joint_pose(self.fixed_beam_id, "j1")
        self.input_origin = self.get_marker_pose(0)

    def load_insertion_poses(self, poses):
        """
        Load approach and target poses for insertion.
        Args:
            poses (dict): A dictionary containing the appraoch and target poses for insertion.
                          E.g. {'b1a': pose_for_approach, 'b1t': pose_for_target}
        """
        for key in poses.keys():
            trans = poses[key][:3, 3]
            new_trans = [trans[0], -trans[2], trans[1]]
            euler = quat2axisangle(mat2quat(poses[key][:3, :3]))
            new_quat = quat2mat(axisangle2quat([euler[0], euler[2], euler[1]]))
            new_pose = make_pose(new_trans, new_quat)

            self.beams[key] = np.dot(self.fixed_beam_origin, new_pose)

    def update_beam_approach_pose(self, beam_id, link_id, insert_end):
        """
        Transform the approach poes of the insertion endpoint to the apporach pose of the grasped link
        """
        group = re.match("(b\d*)(j\d*)", insert_end.name)

        approach_pose_wrt_origin = self.beams["{}a".format(beam_id)]
        beam_info = self.beams[beam_id]

        # local_pose_mat = make_pose((beam_info.link_offsets[link_id] - beam_info.joint_offsets[group[2]]), np.eye(3))
        local_pose_mat = make_pose(
            (beam_info.link_offsets[link_id] - beam_info.joint_offsets["j1"]), np.eye(3)
        )
        self.beams["{}a".format(beam_id)] = np.dot(
            approach_pose_wrt_origin, local_pose_mat
        )

        target_pose_wrt_origin = self.beams["{}t".format(beam_id)]
        beam_info = self.beams[beam_id]
        self.beams["{}t".format(beam_id)] = np.dot(
            target_pose_wrt_origin, local_pose_mat
        )

    def get_marker_pose(self, marker_id):
        """
        Returns a marker pose in the global coordinate system
        Args:
            marker_id (int): ID of a marker
        """

        top_marker_data, wrist_marker_data = None, None
        if "tag_{}".format(marker_id) in self.top_markers:
            top_marker_data = self.top_markers["tag_{}".format(marker_id)]

        if "tag_{}".format(marker_id) in self.wrist_markers:
            wrist_marker_data = self.wrist_markers["tag_{}".format(marker_id)]

        if top_marker_data is not None and wrist_marker_data is None:
            return top_marker_data.pose
        elif top_marker_data is None and wrist_marker_data is not None:
            return wrist_marker_data.pose
        elif top_marker_data is None and wrist_marker_data is None:
            return None
        else:
            if abs((top_marker_data.time - wrist_marker_data.time).seconds) > 5:
                if top_marker_data.time > wrist_marker_data.time:
                    return top_marker_data.pose
                else:
                    return wrist_marker_data.pose
            else:
                # if time difference is short, then prioritise marker pose obserbed by a wrist camera, because it's potentially more accurate.
                return wrist_marker_data.pose

    def construct_scene(self):
        """
        Add boxex around the robot arm to avoid unexpected collisions
        """
        self.robot.movegroup_interface.add_box2scene(
            "table", [0.655, 0, -0.07], [0, 0, 0, 1.0], (1.0, 0.9, 0.1)
        )
        self.robot.movegroup_interface.add_box2scene(
            "back_wall", [-0.4, 0, 0.5], [0, 0, 0, 1.0], (0.1, 1.0, 0.9)
        )
        self.robot.movegroup_interface.add_box2scene(
            "right_wall", [0.655, 0.7, 0.5], [0, 0, 0, 1.0], (1.0, 0.1, 1.0)
        )
        self.robot.movegroup_interface.add_box2scene(
            "left_wall", [0.655, -0.7, 0.5], [0, 0, 0, 1.0], (1.0, 0.1, 1.0)
        )
