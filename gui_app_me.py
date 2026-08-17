import sys
import numpy as np

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg


from arm_model_me import PlanarArm


# ============================================================
# DEFAULT PARAMETERS
# ============================================================

LINK_LENGTHS = [3.0, 2.0, 1.5]

DEFAULT_PICK_POS = (4.0, 2.0)
DEFAULT_PLACE_POS = (-3.0, 3.0)

# Animation parameters
ANIMATION_STEPS = 80
ANIMATION_INTERVAL = 20  # milliseconds


class ArmWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Planar Robotic Arm - Pick & Place GUI")

        # ====================================================
        # ROBOT / STATE PARAMETERS
        # ====================================================

        self.arm = PlanarArm(LINK_LENGTHS)

        # Current joint angles in radians
        self.joint_angles = [0.3, -0.5, -0.3]

        # Dynamic target coordinates
        self.pick_pos = list(DEFAULT_PICK_POS)
        self.place_pos = list(DEFAULT_PLACE_POS)

        # Object tracking state
        self.object_pos = list(self.pick_pos)
        self.holding_object = False

        # ====================================================
        # BUILD GUI
        # ====================================================

        self.init_ui()

        # ====================================================
        # ANIMATION SETUP
        # ====================================================

        self.animation_timer = QtCore.QTimer()
        self.animation_timer.timeout.connect(self.animate_step)

        self.animation_start = None
        self.animation_target = None
        self.animation_step = 0
        self.animation_phase = None

        # Draw initial state
        self.redraw()

    # ========================================================
    # UI SETUP & STYLING
    # ========================================================

    def init_ui(self):
        # Modern Dark Theme Stylesheet
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e2e;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #45475a;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 12px;
                color: #cdd6f4;
                font-size: 13px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 6px;
                left: 10px;
            }
            QLabel {
                color: #cdd6f4;
                font-size: 12px;
            }
            QDoubleSpinBox {
                background-color: #313244;
                color: #cdd6f4;
                border: 1px solid #45475a;
                border-radius: 4px;
                padding: 4px;
                font-size: 12px;
            }
            QSlider::groove:horizontal {
                height: 6px;
                background: #313244;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #89b4fa;
                width: 14px;
                margin: -4px 0;
                border-radius: 7px;
            }
            QPushButton {
                background-color: #89b4fa;
                color: #11111b;
                font-weight: bold;
                border: none;
                border-radius: 6px;
                padding: 10px;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #b4befe;
            }
            QPushButton:disabled {
                background-color: #45475a;
                color: #6c7086;
            }
        """)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(15)
        self.setCentralWidget(central)

        # ----------------------------------------------------
        # PLOT WIDGET
        # ----------------------------------------------------

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#181825')
        self.plot_widget.setXRange(-8, 8)
        self.plot_widget.setYRange(-8, 8)
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)

        layout.addWidget(self.plot_widget, stretch=3)

        # ============================================================
        # MAXIMUM WORKSPACE REACH
        # ============================================================

        max_reach = sum(LINK_LENGTHS)

        theta = np.linspace(
            0,
            2 * np.pi,
            200
        )

        workspace_x = max_reach * np.cos(theta)
        workspace_y = max_reach * np.sin(theta)

        self.workspace_circle = self.plot_widget.plot(
            workspace_x,
            workspace_y,
            pen=pg.mkPen(
                width=2,
                style=QtCore.Qt.DotLine
            )
        )

        # Arm links curve
        self.arm_curve = self.plot_widget.plot(pen=pg.mkPen(color='#89b4fa', width=5))

        # Joint scatter items
        self.joint_scatter = pg.ScatterPlotItem(size=12, brush="#f38ba8", pen=pg.mkPen(None))
        self.plot_widget.addItem(self.joint_scatter)

        # ============================================================
        # POINT LABELS
        # ============================================================

        self.point_labels = []

        # Base + 3 joints + end effector
        for i in range(self.arm.n_joints + 1):

            label = pg.TextItem(
                "",
                anchor=(0.5, 1.5)
            )

            self.plot_widget.addItem(label)

            self.point_labels.append(label)

        # Pick and Place labels
        self.pick_label = pg.TextItem(
            "",
            anchor=(0.5, 1.5)
        )

        self.place_label = pg.TextItem(
            "",
            anchor=(0.5, 1.5)
        )

        self.plot_widget.addItem(
            self.pick_label
        )

        self.plot_widget.addItem(
            self.place_label
        )

        # Pick position marker (Green Circle)
        self.pick_marker = self.plot_widget.plot(
            [self.pick_pos[0]],
            [self.pick_pos[1]],
            pen=None,
            symbol="o",
            symbolBrush="#a6e3a1",
            symbolSize=18,
            name="Pick Location",
        )

        # Place position marker (Blue Square)
        self.place_marker = self.plot_widget.plot(
            [self.place_pos[0]],
            [self.place_pos[1]],
            pen=None,
            symbol="s",
            symbolBrush="#74c7ec",
            symbolSize=18,
            name="Place Location",
        )

        # Object scatter marker (Yellow Circle)
        self.object_scatter = pg.ScatterPlotItem(size=14, brush="#f9e2af")
        self.plot_widget.addItem(self.object_scatter)

        # ----------------------------------------------------
        # CONTROLS PANEL
        # ----------------------------------------------------

        controls_layout = QtWidgets.QVBoxLayout()
        controls_layout.setSpacing(12)
        layout.addLayout(controls_layout, stretch=1)

        # 1. TARGET POSITIONS GROUP
        target_group = QtWidgets.QGroupBox("Target Coordinates")
        target_grid = QtWidgets.QGridLayout()
        target_grid.setSpacing(8)

        # Pick X, Y SpinBoxes
        pick_x_label = QtWidgets.QLabel(
            "<span style='color: #a6e3a1; font-size: 14px;'>●</span> Pick X:"
            )
        target_grid.addWidget(pick_x_label, 0, 0)
        self.pick_x_spin = QtWidgets.QDoubleSpinBox()
        self.pick_x_spin.setRange(-8.0, 8.0)
        self.pick_x_spin.setSingleStep(0.5)
        self.pick_x_spin.setValue(DEFAULT_PICK_POS[0])
        target_grid.addWidget(self.pick_x_spin, 0, 1)

        
        target_grid.addWidget(QtWidgets.QLabel("Pick Y:"), 0, 2)
        self.pick_y_spin = QtWidgets.QDoubleSpinBox()
        self.pick_y_spin.setRange(-8.0, 8.0)
        self.pick_y_spin.setSingleStep(0.5)
        self.pick_y_spin.setValue(DEFAULT_PICK_POS[1])
        target_grid.addWidget(self.pick_y_spin, 0, 3)

        # Place X, Y SpinBoxes
        place_x_label = QtWidgets.QLabel(
                "<span style='color: #74c7ec; font-size: 14px;'>■</span> Place X:"
            )
        target_grid.addWidget(place_x_label, 1, 0)
        self.place_x_spin = QtWidgets.QDoubleSpinBox()
        self.place_x_spin.setRange(-8.0, 8.0)
        self.place_x_spin.setSingleStep(0.5)
        self.place_x_spin.setValue(DEFAULT_PLACE_POS[0])
        target_grid.addWidget(self.place_x_spin, 1, 1)

        target_grid.addWidget(QtWidgets.QLabel("Place Y:"), 1, 2)
        self.place_y_spin = QtWidgets.QDoubleSpinBox()
        self.place_y_spin.setRange(-8.0, 8.0)
        self.place_y_spin.setSingleStep(0.5)
        self.place_y_spin.setValue(DEFAULT_PLACE_POS[1])
        target_grid.addWidget(self.place_y_spin, 1, 3)

        target_group.setLayout(target_grid)
        controls_layout.addWidget(target_group)

        # Connect coordinate change signals
        self.pick_x_spin.valueChanged.connect(self.on_targets_changed)
        self.pick_y_spin.valueChanged.connect(self.on_targets_changed)
        self.place_x_spin.valueChanged.connect(self.on_targets_changed)
        self.place_y_spin.valueChanged.connect(self.on_targets_changed)

        # 2. JOINT CONTROL SLIDERS GROUP
        joints_group = QtWidgets.QGroupBox("Manual Joint Control")
        joints_layout = QtWidgets.QVBoxLayout()
        joints_layout.setSpacing(6)

        self.sliders = []
        self.slider_labels = []

        for i in range(self.arm.n_joints):
            deg_val = int(np.degrees(self.joint_angles[i]))
            lbl = QtWidgets.QLabel(f"Joint {i + 1}: {deg_val}°")
            joints_layout.addWidget(lbl)
            self.slider_labels.append(lbl)

            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            lower = int(np.degrees(self.arm.joint_limits[i][0]))
            upper = int(np.degrees(self.arm.joint_limits[i][1]))
            slider.setRange(lower, upper)
            slider.setValue(deg_val)

            slider.valueChanged.connect(self.on_slider_changed)
            slider.sliderPressed.connect(self.on_slider_pressed)

            joints_layout.addWidget(slider)
            self.sliders.append(slider)

        joints_group.setLayout(joints_layout)
        controls_layout.addWidget(joints_group)

        # 3. STATUS & READOUT GROUP
        info_group = QtWidgets.QGroupBox("Status & Telemetry")
        info_layout = QtWidgets.QVBoxLayout()

        self.ee_label = QtWidgets.QLabel("End effector: (0.00, 0.00)")
        self.status_label = QtWidgets.QLabel("Status: Idle")
        self.status_label.setStyleSheet("color: #a6e3a1; font-weight: bold;")

        info_layout.addWidget(self.ee_label)
        info_layout.addWidget(self.status_label)
        info_group.setLayout(info_layout)
        controls_layout.addWidget(info_group)

        # 4. ACTION BUTTON
        self.run_button = QtWidgets.QPushButton("Run Pick and Place Sequence")
        self.run_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.run_button.clicked.connect(self.on_run_pick_and_place)
        controls_layout.addWidget(self.run_button)

        controls_layout.addStretch()

    # ========================================================
    # TARGET COORDINATES UPDATER
    # ========================================================

    def on_targets_changed(self):
        """Update target coordinates and reposition plot markers live."""
        self.pick_pos = [self.pick_x_spin.value(), self.pick_y_spin.value()]
        self.place_pos = [self.place_x_spin.value(), self.place_y_spin.value()]

        # Reposition target markers in the plot
        self.pick_marker.setData([self.pick_pos[0]], [self.pick_pos[1]])
        self.place_marker.setData([self.place_pos[0]], [self.place_pos[1]])

        # If not animating or holding object, align object with new pick position
        if not self.holding_object and not self.animation_timer.isActive():
            self.object_pos = list(self.pick_pos)
            self.redraw()

    # ========================================================
    # SLIDER CONTROLS
    # ========================================================

    def on_slider_changed(self):
        if self.animation_timer.isActive():
            return

        self.joint_angles = [
            np.radians(slider.value()) for slider in self.sliders
        ]

        # Update angle text labels
        for i, slider in enumerate(self.sliders):
            self.slider_labels[i].setText(f"Joint {i + 1}: {slider.value()}°")

        self.redraw()
        self.status_label.setText("Status: Manual Control")

    def on_slider_pressed(self):
        if self.animation_timer.isActive():
            self.animation_timer.stop()
            self.status_label.setText("Status: Manual Control")
            self.run_button.setEnabled(True)

    # ========================================================
    # REDRAW
    # ========================================================

    def redraw(self):
        points = self.arm.forward_kinematics(self.joint_angles)
        xs, ys = zip(*points)

        # Update arm geometry
        self.arm_curve.setData(xs, ys)
        self.joint_scatter.setData(xs, ys)

        # ============================================================
        # UPDATE JOINT LABELS
        # ============================================================

        labels = [
            "B",
            "J1",
            "J2",
            "EE"
        ]

        for i, (x, y) in enumerate(points):

            if i == len(points) - 1:

                # Last point = End Effector
                text = "EE"

            else:

                text = labels[i]

            self.point_labels[i].setText(text)

            self.point_labels[i].setPos(
                x,
                y
            )

        # Update object position if held
        if self.holding_object:
            self.object_pos = list(self.arm.end_effector(self.joint_angles))

        self.object_scatter.setData([self.object_pos[0]], [self.object_pos[1]])

        # Telemetry readout
        ee = self.arm.end_effector(self.joint_angles)
        self.ee_label.setText(f"End Effector: ({ee[0]:.2f}, {ee[1]:.2f})")

    # ========================================================
    # PICK AND PLACE EXECUTION
    # ========================================================

    def on_run_pick_and_place(self):
        if self.animation_timer.isActive():
            return

        # Fetch active targets
        self.pick_pos = [self.pick_x_spin.value(), self.pick_y_spin.value()]
        self.place_pos = [self.place_x_spin.value(), self.place_y_spin.value()]

        # Reset object state to pick point
        self.object_pos = list(self.pick_pos)
        self.holding_object = False
        self.redraw()

        self.status_label.setText("Status: Calculating Pick IK...")
        self.run_button.setEnabled(False)

        # Inverse Kinematics for Pick Location
        try:
            pick_angles = self.arm.inverse_kinematics(
                self.pick_pos, initial_guess=self.joint_angles
            )
        except ValueError as e:
            self.status_label.setText(f"Error: {e}")
            self.run_button.setEnabled(True)
            return

        self.start_animation(pick_angles, "moving_to_pick")

    # ========================================================
    # ANIMATION ENGINE
    # ========================================================

    def start_animation(self, target_angles, phase):
        self.animation_start = np.array(self.joint_angles, dtype=float)
        self.animation_target = np.array(target_angles, dtype=float)
        self.animation_step = 0
        self.animation_phase = phase

        self.status_label.setText(f"Status: {phase.replace('_', ' ').capitalize()}")
        self.animation_timer.start(ANIMATION_INTERVAL)

    def animate_step(self):
        self.animation_step += 1
        t = min(self.animation_step / ANIMATION_STEPS, 1.0)

        # Smooth cubic interpolation (Smoothstep)
        smooth_t = 3 * t**2 - 2 * t**3
        new_angles = (
            self.animation_start
            + smooth_t * (self.animation_target - self.animation_start)
        )

        # Extract lower and upper limits in radians
        lower_limits = [lim[0] for lim in self.arm.joint_limits]
        upper_limits = [lim[1] for lim in self.arm.joint_limits]

        # Clamp new_angles strictly to joint_limits
        clamped_angles = np.clip(new_angles, lower_limits, upper_limits)
        self.joint_angles = clamped_angles.tolist()

        # Update sliders silently
        for i, slider in enumerate(self.sliders):
            slider.blockSignals(True)
            deg_val = int(np.degrees(self.joint_angles[i]))
            slider.setValue(deg_val)
            self.slider_labels[i].setText(f"Joint {i + 1}: {deg_val}°")
            slider.blockSignals(False)

        self.redraw()

        if t >= 1.0:
            self.animation_timer.stop()
            self.animation_finished()

    def animation_finished(self):
        # PHASE 1: Arrived at Pick
        if self.animation_phase == "moving_to_pick":
            self.status_label.setText("Status: Object Picked")
            self.holding_object = True
            self.object_pos = list(self.arm.end_effector(self.joint_angles))
            self.redraw()

            # IK calculation for Place Location
            try:
                place_angles = self.arm.inverse_kinematics(
                    self.place_pos, initial_guess=self.joint_angles
                )
            except ValueError as e:
                self.status_label.setText(f"Error (Place IK): {e}")
                self.run_button.setEnabled(True)
                return

            QtCore.QTimer.singleShot(
                500,
                lambda: self.start_animation(place_angles, "moving_to_place"),
            )

        # PHASE 2: Arrived at Place
        elif self.animation_phase == "moving_to_place":
            self.status_label.setText("Status: Object Released")
            self.holding_object = False
            self.object_pos = list(self.arm.end_effector(self.joint_angles))
            self.redraw()

            self.animation_phase = None
            self.run_button.setEnabled(True)
            self.status_label.setText("Status: Sequence Complete")


# ============================================================
# MAIN
# ============================================================

def main():
    app = QtWidgets.QApplication(sys.argv)
    win = ArmWindow()
    win.resize(950, 650)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()