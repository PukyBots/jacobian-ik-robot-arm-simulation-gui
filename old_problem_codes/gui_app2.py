"""
Pick-and-place GUI for the planar robotic arm.

Features:
    - Manual joint control using sliders
    - Live end-effector position
    - Jacobian-based inverse kinematics
    - Smooth pick-and-place animation
    - Object follows the end effector while holding
    - GUI remains responsive during movement
"""

import sys
import numpy as np

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

from arm_model_joint_angle_far import PlanarArm


# ============================================================
# ROBOT / TASK PARAMETERS
# ============================================================

LINK_LENGTHS = [3.0, 2.0, 1.5]

PICK_POS = (4.0, 2.0)
PLACE_POS = (-3.0, 3.0)

# Animation
ANIMATION_STEPS = 80
ANIMATION_INTERVAL = 20  # milliseconds


class ArmWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Pick & Place Exercise")

        # ====================================================
        # CREATE ARM
        # ====================================================

        self.arm = PlanarArm(LINK_LENGTHS)

        # Current joint angles in radians
        self.joint_angles = [
            0.3,
            -0.5,
            -0.3
        ]

        # ====================================================
        # CENTRAL WIDGET
        # ====================================================

        central = QtWidgets.QWidget()

        layout = QtWidgets.QHBoxLayout(central)

        self.setCentralWidget(central)

        # ====================================================
        # PLOT
        # ====================================================

        self.plot_widget = pg.PlotWidget()

        self.plot_widget.setXRange(-8, 8)
        self.plot_widget.setYRange(-8, 8)

        self.plot_widget.setAspectLocked(True)

        layout.addWidget(
            self.plot_widget,
            stretch=3
        )

        # ====================================================
        # ARM LINE
        # ====================================================

        self.arm_curve = self.plot_widget.plot(
            pen=pg.mkPen(width=4)
        )

        # ====================================================
        # JOINTS
        # ====================================================

        self.joint_scatter = pg.ScatterPlotItem(
            size=10,
            brush="r"
        )

        self.plot_widget.addItem(
            self.joint_scatter
        )

        # ====================================================
        # PICK POSITION
        # ====================================================

        self.plot_widget.plot(
            [PICK_POS[0]],
            [PICK_POS[1]],
            pen=None,
            symbol="o",
            symbolBrush="g",
            symbolSize=18,
            name="pick bin",
        )

        # ====================================================
        # PLACE POSITION
        # ====================================================

        self.plot_widget.plot(
            [PLACE_POS[0]],
            [PLACE_POS[1]],
            pen=None,
            symbol="s",
            symbolBrush="b",
            symbolSize=18,
            name="place target",
        )

        # ====================================================
        # OBJECT
        # ====================================================

        self.object_scatter = pg.ScatterPlotItem(
            size=14,
            brush="y"
        )

        self.plot_widget.addItem(
            self.object_scatter
        )

        # Object initially starts at PICK position
        self.object_pos = list(PICK_POS)

        # Is the robot currently holding the object?
        self.holding_object = False

        # ====================================================
        # CONTROLS
        # ====================================================

        controls = QtWidgets.QVBoxLayout()

        layout.addLayout(
            controls,
            stretch=1
        )

        # ====================================================
        # SLIDERS
        # ====================================================

        self.sliders = []

        for i in range(self.arm.n_joints):

            controls.addWidget(
                QtWidgets.QLabel(
                    f"Joint {i + 1}"
                )
            )

            slider = QtWidgets.QSlider(
                QtCore.Qt.Horizontal
            )

            lower = int(
                np.degrees(
                    self.arm.joint_limits[i][0]
                )
            )

            upper = int(
                np.degrees(
                    self.arm.joint_limits[i][1]
                )
            )

            slider.setRange(
                lower,
                upper
            )


            slider.setValue(
                int(
                    np.degrees(
                        self.joint_angles[i]
                    )
                )
            )

            slider.valueChanged.connect(
                self.on_slider_changed
            )

               # User started touching slider
            slider.sliderPressed.connect(
                self.on_slider_pressed
            )

            controls.addWidget(slider)

            self.sliders.append(slider)

        # ====================================================
        # END EFFECTOR LABEL
        # ====================================================

        self.ee_label = QtWidgets.QLabel(
            "End effector: (?, ?)"
        )

        controls.addWidget(
            self.ee_label
        )

        # ====================================================
        # STATUS LABEL
        # ====================================================

        self.status_label = QtWidgets.QLabel(
            "Status: idle"
        )

        controls.addWidget(
            self.status_label
        )

        # ====================================================
        # RUN BUTTON
        # ====================================================

        self.run_button = QtWidgets.QPushButton(
            "Run Pick && Place"
        )

        self.run_button.clicked.connect(
            self.on_run_pick_and_place
        )

        controls.addWidget(
            self.run_button
        )

        controls.addStretch()

        # ====================================================
        # ANIMATION VARIABLES
        # ====================================================

        self.animation_timer = QtCore.QTimer()

        self.animation_timer.timeout.connect(
            self.animate_step
        )

        self.animation_start = None
        self.animation_target = None

        self.animation_step = 0

        self.animation_phase = None

        # ====================================================
        # INITIAL DRAW
        # ====================================================

        self.redraw()

    # ========================================================
    # SLIDER CONTROL
    # ========================================================

    def on_slider_changed(self):

        # Don't allow manual control while animation is running
        if self.animation_timer.isActive():
            return

        self.joint_angles = [
            np.radians(
                slider.value()
            )
            for slider in self.sliders
        ]

        self.redraw()

        self.status_label.setText(
            "Status: manual control"
        )

    def on_slider_pressed(self):
   
        if self.animation_timer.isActive():

            self.animation_timer.stop()

            self.status_label.setText(
                "Status: manual control"
            )

            self.run_button.setEnabled(
                True
            )

    # ========================================================
    # REDRAW
    # ========================================================

    def redraw(self):

        # Calculate all joint positions
        points = self.arm.forward_kinematics(
            self.joint_angles
        )

        # Separate X and Y
        xs, ys = zip(*points)

        # Draw arm
        self.arm_curve.setData(
            xs,
            ys
        )

        # Draw joints
        self.joint_scatter.setData(
            xs,
            ys
        )

        # ====================================================
        # OBJECT
        # ====================================================

        if self.holding_object:

            # Object follows end effector
            self.object_pos = list(
                self.arm.end_effector(
                    self.joint_angles
                )
            )

        self.object_scatter.setData(
            [self.object_pos[0]],
            [self.object_pos[1]]
        )

        # ====================================================
        # END EFFECTOR POSITION
        # ====================================================

        ee = self.arm.end_effector(
            self.joint_angles
        )

        self.ee_label.setText(
            f"End effector: "
            f"({ee[0]:.2f}, {ee[1]:.2f})"
        )

    # ========================================================
    # PICK AND PLACE
    # ========================================================

    def on_run_pick_and_place(self):

        # Don't start another animation
        if self.animation_timer.isActive():
            return

         # Reset object to the green pick position
        self.object_pos = list(PICK_POS)

        # Object is not being held yet
        self.holding_object = False

        # Redraw object at pick position
        self.redraw()

        self.status_label.setText(
            "Status: calculating pick position..."
        )

        # ====================================================
        # 1. CALCULATE IK FOR PICK POSITION
        # ====================================================
        try:

            pick_angles = self.arm.inverse_kinematics(
                PICK_POS,
                initial_guess=self.joint_angles
            )

        except ValueError as e:

            self.status_label.setText(
                f"Error: {e}"
            )

            return
    
        # ====================================================
        # 2. START MOVING TO PICK
        # ====================================================

        self.start_animation(
            pick_angles,
            "moving_to_pick"
        )

    # ========================================================
    # START ANIMATION
    # ========================================================

    def start_animation(
        self,
        target_angles,
        phase
    ):

        self.animation_start = np.array(
            self.joint_angles,
            dtype=float
        )

        self.animation_target = np.array(
            target_angles,
            dtype=float
        )

        self.animation_step = 0

        self.animation_phase = phase

        self.status_label.setText(
            "Status: moving"
        )

        # Start QTimer
        self.animation_timer.start(
            ANIMATION_INTERVAL
        )

    # ========================================================
    # ANIMATION STEP
    # ========================================================

    def animate_step(self):

        self.animation_step += 1

        # ====================================================
        # CALCULATE ANIMATION PROGRESS
        # ====================================================

        t = (
            self.animation_step /
            ANIMATION_STEPS
        )

        # Keep between 0 and 1
        t = min(
            t,
            1.0
        )

        # ====================================================
        # SMOOTH INTERPOLATION
        # ====================================================

        # Smoothstep:
        #
        # 0 → slowly starts
        # 1 → slowly stops
        #
        smooth_t = (
            3 * t ** 2
            -
            2 * t ** 3
        )

        new_angles = (
            self.animation_start
            +
            smooth_t
            *
            (
                self.animation_target
                -
                self.animation_start
            )
        )

        self.joint_angles = new_angles.tolist()

        # ====================================================
        # UPDATE SLIDERS
        # ====================================================

        # Temporarily block slider signals
        for i, slider in enumerate(
            self.sliders
        ):

            slider.blockSignals(True)

            slider.setValue(
                int(
                    np.degrees(
                        self.joint_angles[i]
                    )
                )
            )

            slider.blockSignals(False)

        # ====================================================
        # REDRAW
        # ====================================================

        self.redraw()

        # ====================================================
        # ANIMATION FINISHED?
        # ====================================================

        if t >= 1.0:

            self.animation_timer.stop()

            self.animation_finished()

    # ========================================================
    # ANIMATION FINISHED
    # ========================================================

    def animation_finished(self):

        # ====================================================
        # PHASE 1:
        # ARRIVED AT PICK
        # ====================================================

        if self.animation_phase == "moving_to_pick":

            self.status_label.setText(
                "Status: object picked"
            )

            # Grab object
            self.holding_object = True

            # Put object exactly at end effector
            self.object_pos = list(
                self.arm.end_effector(
                    self.joint_angles
                )
            )

            self.redraw()

            # =================================================
            # Calculate IK for PLACE position
            # =================================================

            place_angles = self.arm.inverse_kinematics(
                PLACE_POS,
                initial_guess=self.joint_angles
            )

            # Move toward place position
            QtCore.QTimer.singleShot(
                500,
                lambda: self.start_animation(
                    place_angles,
                    "moving_to_place"
                )
            )

        # ====================================================
        # PHASE 2:
        # ARRIVED AT PLACE
        # ====================================================

        elif self.animation_phase == "moving_to_place":

            self.status_label.setText(
                "Status: object released"
            )

            # Release object
            self.holding_object = False

            # Put object at final end-effector position
            self.object_pos = list(
                self.arm.end_effector(
                    self.joint_angles
                )
            )

            self.redraw()

            # Finished
            self.animation_phase = None

            self.run_button.setEnabled(True)

            self.status_label.setText(
                "Status: done"
            )


# ============================================================
# MAIN
# ============================================================

def main():

    app = QtWidgets.QApplication(
        sys.argv
    )

    win = ArmWindow()

    win.resize(
        900,
        600
    )

    win.show()

    sys.exit(
        app.exec_()
    )


if __name__ == "__main__":
    main()