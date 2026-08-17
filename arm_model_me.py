"""
Simple planar 2D robotic arm.

Features:
    - Forward Kinematics
    - Numerical Jacobian
    - Multiple-solution Inverse Kinematics
    - Selects solution with minimum joint movement
    - Handles unreachable targets by projecting to workspace boundary
"""

import numpy as np


class PlanarArm:

    def __init__(self, link_lengths):
        self.link_lengths = list(link_lengths)
        self.n_joints = len(self.link_lengths)

        self.joint_limits = np.radians([
        [0, 180],
        [-120, 120],
        [-120, 120],
    ])

    # ============================================================
    # BASE / GROUND CONSTRAINT
    # ============================================================

    # No part of the arm is allowed below y = 0.
        self.min_y = 0.0

    def within_joint_limits(self, q):

        q = np.asarray(
            q,
            dtype=float
        )

        for i in range(self.n_joints):

            lower = self.joint_limits[i][0]
            upper = self.joint_limits[i][1]

            if q[i] < lower or q[i] > upper:
                return False

        return True

    def arm_above_base(self, q, tolerance=1e-6):

        points = self.forward_kinematics(q)

        for x, y in points:

            if y < self.min_y - tolerance:
                return False

        return True

    # ============================================================
    # FORWARD KINEMATICS
    # ============================================================

    def forward_kinematics(self, joint_angles):

        if len(joint_angles) != self.n_joints:
            raise ValueError(
                f"Expected {self.n_joints} joint angles, "
                f"got {len(joint_angles)}"
            )

        x = 0.0
        y = 0.0
        theta = 0.0

        points = [(x, y)]

        for length, angle in zip(
            self.link_lengths,
            joint_angles
        ):

            theta += angle

            x += length * np.cos(theta)
            y += length * np.sin(theta)

            points.append((x, y))

        return points

    # ============================================================
    # END EFFECTOR
    # ============================================================

    def end_effector(self, joint_angles):

        return self.forward_kinematics(
            joint_angles
        )[-1]

    # ============================================================
    # ANGLE WRAPPING
    # ============================================================

    @staticmethod
    def wrap_angle(angle):

        return (
            angle + np.pi
        ) % (
            2 * np.pi
        ) - np.pi

    # ============================================================
    # NUMERICAL JACOBIAN
    # ============================================================

    def numerical_jacobian(
        self,
        joint_angles,
        delta=1e-6
    ):

        q = np.asarray(
            joint_angles,
            dtype=float
        )

        if len(q) != self.n_joints:
            raise ValueError(
                f"Expected {self.n_joints} joint angles, "
                f"got {len(q)}"
            )

        current_position = np.asarray(
            self.end_effector(q),
            dtype=float
        )

        J = np.zeros(
            (2, self.n_joints)
        )

        for i in range(self.n_joints):

            q_test = q.copy()

            q_test[i] += delta

            new_position = np.asarray(
                self.end_effector(q_test),
                dtype=float
            )

            J[:, i] = (
                new_position -
                current_position
            ) / delta

        return J

    # ============================================================
    # REACHABLE TARGET
    # ============================================================

    def reachable_target(self, target_xy):

        target = np.asarray(
            target_xy,
            dtype=float
        )

        max_reach = sum(
            self.link_lengths
        )

        distance = np.linalg.norm(
            target
        )

        # Already reachable
        if distance <= max_reach:
            return target

        # Target outside workspace
        if distance == 0:
            return target

        # Project onto maximum reach
        reachable = (
            target
            *
            max_reach
            /
            distance
        )

        print(
            f"Target {tuple(target_xy)} "
            f"is outside workspace."
        )

        print(
            f"Using closest reachable target: "
            f"({reachable[0]:.3f}, "
            f"{reachable[1]:.3f})"
        )

        return reachable

    # ============================================================
    # MULTIPLE-SOLUTION INVERSE KINEMATICS
    # ============================================================

    def inverse_kinematics(
        self,
        target_xy,
        initial_guess=None,
        q3_samples=361,
        tolerance=1e-5
    ):
        """
        Find multiple IK solutions and select the solution
        requiring the smallest total joint movement.

        For a 3-link planar arm:

            q1
            q2
            q3

        The target specifies only:

            x
            y

        Therefore there are multiple possible solutions.

        We search over possible q3 values and solve the
        remaining geometry analytically.

        The selected solution minimizes:

            |dq1| + |dq2| + |dq3|

        relative to the current configuration.
        """

        # ========================================================
        # TARGET
        # ========================================================

        target = self.reachable_target(
            target_xy
        )

        # ========================================================
        # CHECK ARM
        # ========================================================

        if self.n_joints != 3:
            raise ValueError(
                "This multiple-solution IK implementation "
                "expects exactly 3 joints."
            )

        L1, L2, L3 = self.link_lengths

        # ========================================================
        # CURRENT JOINT CONFIGURATION
        # ========================================================

        if initial_guess is None:

            current_q = np.zeros(
                3
            )

        else:

            current_q = np.asarray(
                initial_guess,
                dtype=float
            ).copy()

            if len(current_q) != 3:
                raise ValueError(
                    "initial_guess must contain 3 angles"
                )

        # ========================================================
        # TARGET DISTANCE
        # ========================================================

        target_distance = np.linalg.norm(
            target
        )

        if target_distance < tolerance:

            # Target almost at base.
            # Return a configuration close to current.
            return current_q.tolist()

        target_angle = np.arctan2(
            target[1],
            target[0]
        )

        # ========================================================
        # STORE ALL VALID SOLUTIONS
        # ========================================================

        solutions = []

        # ========================================================
        # SEARCH OVER q3
        # ========================================================

        q3_values = np.linspace(
            -np.pi,
            np.pi,
            q3_samples
        )

        for q3 in q3_values:

            # ----------------------------------------------------
            # Combine link 2 and link 3
            # ----------------------------------------------------
            #
            # Links 2 and 3 form an equivalent vector:
            #
            #       L3
            #      /
            #     /
            #    L2
            #
            # Their combined vector has:
            #
            # magnitude = R
            # angle     = beta
            #

            A = (
                L2 +
                L3 * np.cos(q3)
            )

            B = (
                L3 *
                np.sin(q3)
            )

            R = np.sqrt(
                A**2 + B**2
            )

            beta = np.arctan2(
                B,
                A
            )

            # ----------------------------------------------------
            # Check whether target can be reached
            # using L1 and equivalent R.
            # ----------------------------------------------------

            cosine_value = (
                target_distance**2
                -
                L1**2
                -
                R**2
            ) / (
                2 *
                L1 *
                R
            )

            # Numerical protection
            if cosine_value < -1.0:
                if cosine_value < -1.000001:
                    continue

                cosine_value = -1.0

            if cosine_value > 1.0:
                if cosine_value > 1.000001:
                    continue

                cosine_value = 1.0

            # ----------------------------------------------------
            # Two possible elbow configurations
            # ----------------------------------------------------

            delta = np.arccos(
                cosine_value
            )

            for elbow_sign in [
                1,
                -1
            ]:

                delta_signed = (
                    elbow_sign *
                    delta
                )

                # ------------------------------------------------
                # Solve first joint angle
                # ------------------------------------------------

                correction = np.arctan2(
                    R *
                    np.sin(delta_signed),

                    L1 +
                    R *
                    np.cos(delta_signed)
                )

                q1 = (
                    target_angle -
                    correction
                )

                # ------------------------------------------------
                # Absolute angle of equivalent second vector
                # ------------------------------------------------

                theta2 = (
                    q1 +
                    delta_signed
                )

                # ------------------------------------------------
                # Convert absolute angles into relative joints
                # ------------------------------------------------

                q2 = (
                    theta2 -
                    q1
                )

                # q3 is already relative
                q3_candidate = q3

                candidate = np.array([
                    self.wrap_angle(q1),
                    self.wrap_angle(q2),
                    self.wrap_angle(q3_candidate)
                ])

                if not self.within_joint_limits(candidate):
                    continue

                if not self.arm_above_base(candidate):
                    continue

                # =================================================
                # VERIFY CANDIDATE USING FORWARD KINEMATICS
                # =================================================

                actual_position = np.asarray(
                    self.end_effector(candidate)
                )

                position_error = np.linalg.norm(
                    target -
                    actual_position
                )

                if position_error > tolerance:
                    continue

                # =================================================
                # CALCULATE JOINT MOVEMENT
                # =================================================

                angle_difference = (
                    candidate -
                    current_q
                )

                # Important:
                # Calculate shortest angular difference.
                angle_difference = np.array([
                    self.wrap_angle(x)
                    for x in angle_difference
                ])

                # Total joint movement
                movement_cost = np.sum(
                    np.abs(
                        angle_difference
                    )
                )

                # =================================================
                # SAVE SOLUTION
                # =================================================

                solutions.append({
                    "q": candidate,
                    "error": position_error,
                    "movement": movement_cost
                })

        # ========================================================
        # NO SOLUTION FOUND
        # ========================================================

        if len(solutions) == 0:

            print(
                "No analytical IK solution found."
            )

            print(
                "Falling back to Jacobian IK."
            )

            return self.jacobian_ik(
                target,
                current_q
            )

        # ========================================================
        # SORT BY JOINT MOVEMENT
        # ========================================================

        solutions.sort(
            key=lambda solution:
                solution["movement"]
        )

        # ========================================================
        # BEST SOLUTION
        # ========================================================

        best_solution = solutions[0]

        best_q = best_solution["q"]

        print(
            "\nIK solutions found:",
            len(solutions)
        )

        print(
            "Selected solution:"
        )

        print(
            "Joint angles (degrees):",
            np.degrees(best_q)
        )

        print(
            "Total joint movement:",
            best_solution["movement"]
        )

        print(
            "Position error:",
            best_solution["error"]
        )

        return best_q.tolist()

    # ============================================================
    # JACOBIAN FALLBACK
    # ============================================================

    def jacobian_ik(
        self,
        target,
        initial_guess,
        max_iterations=1000,
        tolerance=1e-5,
        damping=0.15,
        step_size=0.5,
        max_joint_step=0.08
    ):
        """
        Damped Jacobian IK used only as a fallback.
        """

        q = np.asarray(
            initial_guess,
            dtype=float
        ).copy()

        best_q = q.copy()

        best_error = float(
            "inf"
        )

        for iteration in range(
            max_iterations
        ):

            current_position = np.asarray(
                self.end_effector(q)
            )

            error = (
                target -
                current_position
            )

            error_norm = np.linalg.norm(
                error
            )

            if error_norm < best_error:

                best_error = error_norm
                best_q = q.copy()

            if error_norm < tolerance:

                return q.tolist()

            # ----------------------------------------------------
            # Jacobian
            # ----------------------------------------------------

            J = self.numerical_jacobian(
                q
            )

            # ----------------------------------------------------
            # Damped pseudoinverse
            # ----------------------------------------------------

            I = np.eye(2)

            J_damped = (
                J.T
                @
                np.linalg.inv(
                    J @ J.T
                    +
                    damping**2 * I
                )
            )

            dq = (
                J_damped
                @
                error
            )

            # ----------------------------------------------------
            # Limit movement
            # ----------------------------------------------------

            dq_norm = np.linalg.norm(
                dq
            )

            if dq_norm > max_joint_step:

                dq *= (
                    max_joint_step /
                    dq_norm
                )

            q += (
                step_size *
                dq
            )

            # ----------------------------------------------------
            # Normalize angles
            # ----------------------------------------------------

            q = np.array([
                self.wrap_angle(x)
                for x in q
            ])

        return best_q.tolist()


# ================================================================
# TEST
# ================================================================

if __name__ == "__main__":

    arm = PlanarArm(
        [3.0, 2.0, 1.5]
    )

    # Current configuration
    current_angles = [
        np.radians(30),
        np.radians(-20),
        np.radians(-10)
    ]

    print(
        "\nCurrent joint angles:"
    )

    print(
        np.degrees(current_angles)
    )

    # ------------------------------------------------------------
    # Forward Kinematics
    # ------------------------------------------------------------

    points = arm.forward_kinematics(
        current_angles
    )

    print(
        "\nCurrent end effector:"
    )

    print(
        points[-1]
    )

    # ------------------------------------------------------------
    # Jacobian
    # ------------------------------------------------------------

    J = arm.numerical_jacobian(
        current_angles
    )

    print(
        "\nJacobian:"
    )

    print(J)

    # ------------------------------------------------------------
    # PICK TARGET
    # ------------------------------------------------------------

    target = (
        4.0,
        2.0
    )

    print(
        "\n=========================="
    )

    print(
        "INVERSE KINEMATICS"
    )

    print(
        "=========================="
    )

    print(
        "Target:",
        target
    )

    solution = arm.inverse_kinematics(
        target,
        initial_guess=current_angles
    )

    # ------------------------------------------------------------
    # Solution
    # ------------------------------------------------------------

    print(
        "\nSelected joint angles:"
    )

    print(
<<<<<<< HEAD
        "Radians:", [f"{x:.2f}" for x in solution]
        )

    print(
        "Degrees:", [f"{x:.2f}" for x in np.degrees(solution)]
        )
=======
<<<<<<< HEAD
        "Radians:",
        solution
    )

    print(
        "Degrees:",
        np.degrees(solution)
    )
=======
        "Radians:", [f"{x:.2f}" for x in solution]
        )

    print(
        "Degrees:", [f"{x:.2f}" for x in np.degrees(solution)]
        )
>>>>>>> 7ebb289 (Added files)
>>>>>>> aab5e08 (Added files)

    # ------------------------------------------------------------
    # Verify
    # ------------------------------------------------------------

    final_position = arm.end_effector(
        solution
    )

    print(
        "\nFinal end effector:"
    )

<<<<<<< HEAD
    print([f"{x:.2f}" for x in final_position])
=======
<<<<<<< HEAD
    print(
        final_position
    )
=======
    print([f"{x:.2f}" for x in final_position])
>>>>>>> 7ebb289 (Added files)
>>>>>>> aab5e08 (Added files)

    # ------------------------------------------------------------
    # Position error
    # ------------------------------------------------------------

    error = np.linalg.norm(
        np.asarray(target)
        -
        np.asarray(final_position)
    )

    print(
<<<<<<< HEAD
        "\nPosition error:" f"{error:.6f}"
=======
<<<<<<< HEAD
        "\nPosition error:",
        error
=======
        "\nPosition error:" f"{error:.6f}"
>>>>>>> 7ebb289 (Added files)
>>>>>>> aab5e08 (Added files)
    )