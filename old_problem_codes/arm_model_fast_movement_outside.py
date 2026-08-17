"""
Simple planar (2D) robotic arm model.

Implements:
    - Forward Kinematics (FK)
    - Jacobian-based Inverse Kinematics (IK)
    - Numerical Jacobian using finite differences
    - Jacobian pseudoinverse for joint updates
"""

import numpy as np


class PlanarArm:
    def __init__(self, link_lengths):
        self.link_lengths = list(link_lengths)
        self.n_joints = len(self.link_lengths)

    # ============================================================
    # FORWARD KINEMATICS
    # ============================================================

    def forward_kinematics(self, joint_angles):
        """
        Calculate the position of every joint and the end effector.

        Parameters
        ----------
        joint_angles : list or numpy array
            Joint angles in radians.
            Each angle is relative to the previous link.

        Returns
        -------
        points : list of tuples
            [(x0, y0), (x1, y1), ..., (xN, yN)]

            First point  = base
            Last point   = end effector
        """

        if len(joint_angles) != self.n_joints:
            raise ValueError(
                f"Expected {self.n_joints} joint angles, "
                f"got {len(joint_angles)}"
            )

        x = 0.0
        y = 0.0
        theta = 0.0

        points = [(x, y)]

        for length, angle in zip(self.link_lengths, joint_angles):

            # Because the angles are relative to the previous link,
            # accumulate them.
            theta += angle

            # Calculate the new joint position
            x += length * np.cos(theta)
            y += length * np.sin(theta)

            points.append((x, y))

        return points

    # ============================================================
    # END EFFECTOR
    # ============================================================

    def end_effector(self, joint_angles):
        """
        Return the current (x, y) position of the end effector.
        """

        return self.forward_kinematics(joint_angles)[-1]

    # ============================================================
    # NUMERICAL JACOBIAN
    # ============================================================

    def numerical_jacobian(self, joint_angles, delta=1e-6):
        """
        Calculate the numerical Jacobian using finite differences.

        The Jacobian describes how small changes in joint angles
        affect the end-effector position.

                  dq
        dx = J * ---
                  dt

        For this planar arm:

            J = [ dx/dq1  dx/dq2  dx/dq3 ... ]
                [ dy/dq1  dy/dq2  dy/dq3 ... ]

        Shape:
            2 x number_of_joints
        """

        q = np.asarray(joint_angles, dtype=float)

        if len(q) != self.n_joints:
            raise ValueError(
                f"Expected {self.n_joints} joint angles, "
                f"got {len(q)}"
            )

        # Current end-effector position
        current_position = np.asarray(
            self.end_effector(q),
            dtype=float
        )

        # Jacobian:
        # 2 rows -> x and y
        # n columns -> one for each joint
        J = np.zeros((2, self.n_joints))

        # Perturb each joint individually
        for i in range(self.n_joints):

            q_test = q.copy()

            # Slightly change joint i
            q_test[i] += delta

            # Calculate new end-effector position
            new_position = np.asarray(
                self.end_effector(q_test),
                dtype=float
            )

            # Finite difference approximation:
            #
            # derivative ≈
            # (new_position - old_position) / delta
            #
            # This becomes column i of the Jacobian.
            J[:, i] = (
                new_position - current_position
            ) / delta

        return J

    # ============================================================
    # INVERSE KINEMATICS
    # ============================================================

    def inverse_kinematics(
        self,
        target_xy,
        initial_guess=None,
        max_iterations=1000,
        tolerance=1e-5,
        step_size=0.5,
    ):
        """
        Calculate joint angles required to reach target (x, y).

        Uses:

            1. Forward kinematics
            2. Cartesian position error
            3. Numerical Jacobian
            4. Jacobian pseudoinverse
            5. Iterative joint updates

        Parameters
        ----------
        target_xy : tuple/list
            Desired end-effector position (x, y).

        initial_guess : list/array, optional
            Starting joint angles in radians.

        max_iterations : int
            Maximum number of IK iterations.

        tolerance : float
            Target position error tolerance.

        step_size : float
            Fraction of the calculated joint update to apply
            at each iteration.

        Returns
        -------
        list
            Joint angles in radians.
        """

        target = np.asarray(target_xy, dtype=float)

        # --------------------------------------------------------
        # Validate target
        # --------------------------------------------------------

        if target.shape != (2,):
            raise ValueError(
                "target_xy must contain exactly two values: (x, y)"
            )

        # --------------------------------------------------------
        # Initial joint configuration
        # --------------------------------------------------------

        if initial_guess is None:

            # Start with all joints at zero
            q = np.zeros(self.n_joints)

        else:

            q = np.asarray(
                initial_guess,
                dtype=float
            ).copy()

            if len(q) != self.n_joints:
                raise ValueError(
                    f"initial_guess must contain "
                    f"{self.n_joints} joint angles"
                )

        # --------------------------------------------------------
        # Keep track of the best solution
        # --------------------------------------------------------

        best_q = q.copy()

        best_error = float("inf")

        # --------------------------------------------------------
        # IK ITERATION
        # --------------------------------------------------------

        for iteration in range(max_iterations):

            # ----------------------------------------------------
            # 1. Calculate current end-effector position
            # ----------------------------------------------------

            current_position = np.asarray(
                self.end_effector(q),
                dtype=float
            )

            # ----------------------------------------------------
            # 2. Calculate Cartesian error
            # ----------------------------------------------------

            error = target - current_position

            error_norm = np.linalg.norm(error)

            # Keep the best solution found so far
            if error_norm < best_error:

                best_error = error_norm
                best_q = q.copy()

            # ----------------------------------------------------
            # 3. Check if target has been reached
            # ----------------------------------------------------

            if error_norm < tolerance:

                return q.tolist()

            # ----------------------------------------------------
            # 4. Calculate Jacobian
            # ----------------------------------------------------

            J = self.numerical_jacobian(q)

            # ----------------------------------------------------
            # 5. Calculate Jacobian pseudoinverse
            # ----------------------------------------------------

            J_pinv = np.linalg.pinv(J)

            # ----------------------------------------------------
            # 6. Calculate required joint change
            # ----------------------------------------------------

            dq = J_pinv @ error

            # ----------------------------------------------------
            # 7. Apply only part of the calculated movement
            # ----------------------------------------------------

            q += step_size * dq

        # --------------------------------------------------------
        # Maximum iterations reached
        # --------------------------------------------------------

        # Return the best solution found.
        return best_q.tolist()


# ================================================================
# SIMPLE TEST
# ================================================================

if __name__ == "__main__":

    # Create a 3-joint planar arm
    arm = PlanarArm([3.0, 2.0, 1.5])

    # ------------------------------------------------------------
    # Test Forward Kinematics
    # ------------------------------------------------------------

    test_angles = [
        np.radians(30),
        np.radians(20),
        np.radians(-10),
    ]

    points = arm.forward_kinematics(test_angles)

    print("\nForward Kinematics")
    print("------------------")

    for i, point in enumerate(points):
        print(f"Point {i}: ({point[0]:.4f}, {point[1]:.4f})")

    print(
        f"End effector: "
        f"({points[-1][0]:.4f}, {points[-1][1]:.4f})"
    )

    # ------------------------------------------------------------
    # Test Jacobian
    # ------------------------------------------------------------

    J = arm.numerical_jacobian(test_angles)

    print("\nJacobian")
    print("--------")
    print(J)

    # ------------------------------------------------------------
    # Test Inverse Kinematics
    # ------------------------------------------------------------

    target = (4.0, 2.0)

    solution = arm.inverse_kinematics(
        target,
        initial_guess=test_angles,
    )

    print("\nInverse Kinematics")
    print("------------------")

    print("Target:", target)

    print("Joint angles (radians):")
    print(solution)

    print("\nJoint angles (degrees):")
    print(np.degrees(solution))

    # ------------------------------------------------------------
    # Verify IK result using FK
    # ------------------------------------------------------------

    final_position = arm.end_effector(solution)

    print("\nIK Verification")
    print("----------------")
    print(
        f"Calculated end effector: "
        f"({final_position[0]:.6f}, "
        f"{final_position[1]:.6f})"
    )

    error = np.linalg.norm(
        np.asarray(target) -
        np.asarray(final_position)
    )

    print(f"Position error: {error:.8f}")