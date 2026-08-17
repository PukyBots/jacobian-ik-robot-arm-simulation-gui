"""
Simple planar (2D) robotic arm model.

This is a simplified stand-in for the real 6-DOF arm the role works with --
same underlying ideas (forward kinematics, inverse kinematics, joint-space
motion), much less setup. The README has a short question about how this
maps onto a real ROS2/Gazebo robot.
"""

import numpy as np


class PlanarArm:
    def __init__(self, link_lengths):
        self.link_lengths = list(link_lengths)
        self.n_joints = len(self.link_lengths)

    def forward_kinematics(self, joint_angles):
        """joint_angles: list of n_joints angles in radians, each relative to
        the previous link's direction.
        Returns a list of (x, y) points: base, then each joint, ending at
        the end effector.
        """
        assert len(joint_angles) == self.n_joints, "wrong number of joint angles"
        x, y, theta = 0.0, 0.0, 0.0
        points = [(x, y)]
        for length, angle in zip(self.link_lengths, joint_angles):
            theta += angle
            x += length * np.cos(theta)
            y += length * np.sin(theta)
            points.append((x, y))
        return points

    def end_effector(self, joint_angles):
        return self.forward_kinematics(joint_angles)[-1]

    def inverse_kinematics(self, target_xy, initial_guess=None):
        """
        TODO (candidate): implement this.

        Given a target (x, y) for the end effector, return joint angles
        (radians, length n_joints) that reach it -- or get as close as
        possible, if it's out of reach.

        Any approach is fine: closed-form geometry, Jacobian/gradient
        descent, CCD, iterative search, whatever you're comfortable
        reasoning about and can explain. Using references or AI tools is
        fine -- we care whether you understand what your solution does and
        why, not where the idea came from.
        """
        raise NotImplementedError
