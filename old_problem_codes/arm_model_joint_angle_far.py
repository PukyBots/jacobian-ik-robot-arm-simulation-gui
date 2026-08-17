import numpy as np


class PlanarArm:
    def __init__(self, link_lengths):
        self.link_lengths = list(link_lengths)
        self.n_joints = len(self.link_lengths)

    def forward_kinematics(self, joint_angles):

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
        damping=0.15,
        max_joint_step=0.08,
        max_joint_angle=np.pi,

    ):

        target = np.asarray(target_xy, dtype=float)

        # --------------------------------------------------------
        # Validate target
        # --------------------------------------------------------

        if target.shape != (2,):
            raise ValueError(
                "target_xy must contain exactly two values: (x, y)"
            )

         # ========================================================
        # REACHABILITY CHECK
        # ========================================================

        # ========================================================

        max_reach = sum(self.link_lengths)

        target_distance = np.linalg.norm(target)

        # If target is outside the arm's maximum reach,
        # project it onto the maximum reach circle.
        if target_distance > max_reach:

            print(
                f"Target {tuple(target_xy)} is outside "
                f"maximum reach."
            )

            print(
                f"Target distance: {target_distance:.2f}"
            )

            print(
                f"Maximum reach: {max_reach:.2f}"
            )

            # Scale target back to maximum reachable distance
            target = (
                target
                *
                max_reach
                /
                target_distance
            )

            print(
                f"Using closest reachable target: "
                f"({target[0]:.2f}, {target[1]:.2f})"
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

             # ====================================================
            # DAMPED LEAST SQUARES
            # ====================================================

            #
            # dq =
            #
            # J.T @ inv(
            #       J @ J.T +
            #       lambda^2 I
            # )
            # @ error
            #

            identity = np.eye(2)

            damping_matrix = (
                damping ** 2
            ) * identity

            try:

                dq = (
                    J.T
                    @
                    np.linalg.inv(
                        J @ J.T +
                        damping_matrix
                    )
                    @
                    error
                )

            except np.linalg.LinAlgError:

                # Extremely unlikely, but safely
                # stop if matrix becomes singular.

                break

            # ====================================================
            # LIMIT JOINT MOVEMENT
            # ====================================================

            dq_norm = np.linalg.norm(
                dq
            )

            if dq_norm > max_joint_step:

                dq = (
                    dq
                    *
                    max_joint_step
                    /
                    dq_norm
                )

            # ====================================================
            # UPDATE JOINT ANGLES
            # ====================================================

            q += step_size * dq

            # ====================================================
            # LIMIT JOINT ANGLES
            # ====================================================

            q = np.clip(
                q,
                -max_joint_angle,
                max_joint_angle
            )

        # ========================================================
        # ITERATIONS FINISHED
        # ========================================================

        print(
            f"IK did not reach target exactly.\n"
            f"Best position error: "
            f"{best_error:.6f}"
        )

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
