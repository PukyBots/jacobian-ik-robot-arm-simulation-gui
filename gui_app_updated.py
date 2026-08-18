import sys
import numpy as np

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

from arm_model_updated import PlanarArm

# ============================================================
# DEFAULT PARAMETERS
# ============================================================

LINK_LENGTHS = [3.0, 2.0, 1.5]

DEFAULT_PICK_POS = (4.0, 2.0)
DEFAULT_PLACE_POS = (-3.0, 3.0)

ANIMATION_STEPS = 80
ANIMATION_INTERVAL = 20  # milliseconds


class ArmWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Planar Robotic Arm - Pick & Place GUI")

        # Robot & State Parameters
        self.arm = PlanarArm(LINK_LENGTHS)
        self.joint_angles = [0.3, -0.5, -0.3]

        self.pick_pos = list(DEFAULT_PICK_POS)
        self.place_pos = list(DEFAULT_PLACE_POS)

        self.object_pos = list(self.pick_pos)
        self.holding_object = False

        self.init_ui()

        # Animation Timer Setup
        self.animation_timer = QtCore.QTimer()
        self.animation_timer.timeout.connect(self.animate_step)

        self.animation_start = None
        self.animation_target = None
        self.animation_step = 0
        self.animation_phase = None

        self.redraw()

    def init_ui(self):
        # Modern Dark Theme Stylesheet (Catppuccin Mocha palette)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e2e;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #45475a;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 14px;
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
            QTextEdit {
                background-color: #313244;
                color: #a6e3a1;
                border: 1px solid #45475a;
                border-radius: 6px;
                font-family: monospace;
                font-size: 11px;
                padding: 4px;
            }
        """)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        layout.setContentsMargins(15, 15, 15, 15)
        layout.setSpacing(15)
        self.setCentralWidget(central)

        # Plot Widget Setup
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#181825')
        self.plot_widget.setXRange(-8, 8)
        self.plot_widget.setYRange(-8, 8)
        self.plot_widget.setAspectLocked(True)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        layout.addWidget(self.plot_widget, stretch=3)

        # Maximum Workspace Reach Visualizer
        max_reach = sum(LINK_LENGTHS)
        theta = np.linspace(0, 2 * np.pi, 200)
        workspace_x = max_reach * np.cos(theta)
        workspace_y = max_reach * np.sin(theta)

        self.workspace_circle = self.plot_widget.plot(
            workspace_x, workspace_y,
            pen=pg.mkPen(color='#45475a', width=2, style=QtCore.Qt.DotLine)
        )

        # Arm links & joints
        self.arm_curve = self.plot_widget.plot(pen=pg.mkPen(color='#89b4fa', width=5))
        self.joint_scatter = pg.ScatterPlotItem(size=12, brush="#f38ba8", pen=pg.mkPen(None))
        self.plot_widget.addItem(self.joint_scatter)

        # Point Labels (Base, Joints, EE)
        self.point_labels = []
        for i in range(self.arm.n_joints + 1):
            label = pg.TextItem("", anchor=(0.5, 1.8))
            self.plot_widget.addItem(label)
            self.point_labels.append(label)

        # Pick and Place Markers & Object
        self.pick_marker = self.plot_widget.plot(
            [self.pick_pos[0]], [self.pick_pos[1]],
            pen=None, symbol="o", symbolBrush="#a6e3a1", symbolSize=16, name="Pick"
        )
        self.place_marker = self.plot_widget.plot(
            [self.place_pos[0]], [self.place_pos[1]],
            pen=None, symbol="s", symbolBrush="#74c7ec", symbolSize=16, name="Place"
        )
        self.object_scatter = pg.ScatterPlotItem(size=14, brush="#f9e2af", symbol="s")
        self.plot_widget.addItem(self.object_scatter)

        # ----------------------------------------------------
        # CONTROLS PANEL
        # ----------------------------------------------------
        controls_layout = QtWidgets.QVBoxLayout()
        controls_layout.setSpacing(12)
        layout.addLayout(controls_layout, stretch=1)

        # 1. Target Coordinates Group
        target_group = QtWidgets.QGroupBox("Target Coordinates")
        target_grid = QtWidgets.QGridLayout()
        target_grid.setSpacing(8)

        pick_x_label = QtWidgets.QLabel("<span style='color: #a6e3a1;'>●</span> Pick X:")
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

        place_x_label = QtWidgets.QLabel("<span style='color: #74c7ec;'>■</span> Place X:")
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

        self.pick_x_spin.valueChanged.connect(self.on_targets_changed)
        self.pick_y_spin.valueChanged.connect(self.on_targets_changed)
        self.place_x_spin.valueChanged.connect(self.on_targets_changed)
        self.place_y_spin.valueChanged.connect(self.on_targets_changed)

        # 2. Manual Joint Control Sliders
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

        # 3. Status & Telemetry Group
        info_group = QtWidgets.QGroupBox("Status & Telemetry")
        info_layout = QtWidgets.QVBoxLayout()

        self.ee_label = QtWidgets.QLabel("End Effector: (0.00, 0.00)")
        self.status_label = QtWidgets.QLabel("Status: Idle")
        self.status_label.setStyleSheet("color: #a6e3a1; font-weight: bold;")

        info_layout.addWidget(self.ee_label)
        info_layout.addWidget(self.status_label)
        info_group.setLayout(info_layout)
        controls_layout.addWidget(info_group)

        # 4. Action Buttons
        button_layout = QtWidgets.QHBoxLayout()
        self.run_button = QtWidgets.QPushButton("Run Sequence")
        self.run_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.run_button.clicked.connect(self.on_run_pick_and_place)
        button_layout.addWidget(self.run_button)

        self.home_button = QtWidgets.QPushButton("Reset Home")
        self.home_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.home_button.setStyleSheet("background-color: #f38ba8; color: #11111b;")
        self.home_button.clicked.connect(self.reset_home)
        button_layout.addWidget(self.home_button)
        controls_layout.addLayout(button_layout)

        # 5. IK Results Box
        controls_layout.addWidget(QtWidgets.QLabel("IK Calculation Results:"))
        self.results_box = QtWidgets.QTextEdit()
        self.results_box.setReadOnly(True)
        self.results_box.setFixedHeight(140)
        controls_layout.addWidget(self.results_box)

        controls_layout.addStretch()

    def show_ik_results(self, target, solution, final_position):
        """Display IK calculation results and error metrics in the GUI."""
        error = np.linalg.norm(np.asarray(target) - np.asarray(final_position))
        radians_text = [f"{x:.2f}" for x in solution]
        degrees_text = [f"{x:.2f}" for x in np.degrees(solution)]
        final_text = [f"{x:.2f}" for x in final_position]

        text = (
            f"Target: [{target[0]:.2f}, {target[1]:.2f}]\n"
            f"Joint Radians: {radians_text}\n"
            f"Joint Degrees: {degrees_text}\n"
            f"Achieved EE: [{final_text[0]}, {final_text[1]}]\n"
            f"Position Error: {error:.6f}"
        )
        self.results_box.setPlainText(text)

    def on_targets_changed(self):
        """Update target coordinates and reposition plot markers live."""
        self.pick_pos = [self.pick_x_spin.value(), self.pick_y_spin.value()]
        self.place_pos = [self.place_x_spin.value(), self.place_y_spin.value()]

        self.pick_marker.setData([self.pick_pos[0]], [self.pick_pos[1]])
        self.place_marker.setData([self.place_pos[0]], [self.place_pos[1]])

        if not self.holding_object and not self.animation_timer.isActive():
            self.object_pos = list(self.pick_pos)
            self.redraw()

    def on_slider_changed(self):
        if self.animation_timer.isActive():
            return

        self.joint_angles = [np.radians(slider.value()) for slider in self.sliders]
        for i, slider in enumerate(self.sliders):
            self.slider_labels[i].setText(f"Joint {i + 1}: {slider.value()}°")

        self.redraw()
        self.status_label.setText("Status: Manual Control")

    def on_slider_pressed(self):
        if self.animation_timer.isActive():
            self.animation_timer.stop()
            self.status_label.setText("Status: Manual Control (Interrupted)")
            self.run_button.setEnabled(True)

    def reset_home(self):
        if self.animation_timer.isActive():
            self.animation_timer.stop()
        self.joint_angles = [0.0, 0.0, 0.0]
        self.holding_object = False
        self.object_pos = list(self.pick_pos)
        
        for i, slider in enumerate(self.sliders):
            slider.blockSignals(True)
            slider.setValue(0)
            self.slider_labels[i].setText(f"Joint {i + 1}: 0°")
            slider.blockSignals(False)

        self.redraw()
        self.status_label.setText("Status: Reset to Home")
        self.run_button.setEnabled(True)

    def redraw(self):
        points = self.arm.forward_kinematics(self.joint_angles)
        xs, ys = zip(*points)

        self.arm_curve.setData(xs, ys)
        self.joint_scatter.setData(xs, ys)

        labels = ["Base", "Joint 1", "Joint 2", "End Effector"]
        for i, (x, y) in enumerate(points):
            text = "EE" if i == len(points) - 1 else labels[i]
            self.point_labels[i].setText(text)
            self.point_labels[i].setPos(x, y)

        if self.holding_object:
            self.object_pos = list(self.arm.end_effector(self.joint_angles))

        self.object_scatter.setData([self.object_pos[0]], [self.object_pos[1]])

        ee = self.arm.end_effector(self.joint_angles)
        self.ee_label.setText(f"End Effector: ({ee[0]:.2f}, {ee[1]:.2f})")

    def on_run_pick_and_place(self):
        if self.animation_timer.isActive():
            return

        self.pick_pos = [self.pick_x_spin.value(), self.pick_y_spin.value()]
        self.place_pos = [self.place_x_spin.value(), self.place_y_spin.value()]

        self.object_pos = list(self.pick_pos)
        self.holding_object = False
        self.redraw()

        self.status_label.setText("Status: Calculating Pick IK...")
        self.run_button.setEnabled(False)

        try:
            pick_angles = self.arm.inverse_kinematics(
                self.pick_pos, initial_guess=self.joint_angles
            )
            final_position = self.arm.end_effector(pick_angles)
            self.show_ik_results(self.pick_pos, pick_angles, final_position)
        except ValueError as e:
            self.status_label.setText(f"Error: {e}")
            self.run_button.setEnabled(True)
            return

        self.start_animation(pick_angles, "moving_to_pick")

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

        # Smoothstep interpolation
        smooth_t = 3 * t**2 - 2 * t**3
        new_angles = (
            self.animation_start
            + smooth_t * (self.animation_target - self.animation_start)
        )

        lower_limits = [lim[0] for lim in self.arm.joint_limits]
        upper_limits = [lim[1] for lim in self.arm.joint_limits]

        clamped_angles = np.clip(new_angles, lower_limits, upper_limits)
        self.joint_angles = clamped_angles.tolist()

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
        if self.animation_phase == "moving_to_pick":
            self.status_label.setText("Status: Object Picked")
            self.holding_object = True
            self.object_pos = list(self.arm.end_effector(self.joint_angles))
            self.redraw()

            try:
                place_angles = self.arm.inverse_kinematics(
                    self.place_pos, initial_guess=self.joint_angles
                )
                final_position = self.arm.end_effector(place_angles)
                self.show_ik_results(self.place_pos, place_angles, final_position)
            except ValueError as e:
                self.status_label.setText(f"Error (Place IK): {e}")
                self.run_button.setEnabled(True)
                return

            QtCore.QTimer.singleShot(
                400,
                lambda: self.start_animation(place_angles, "moving_to_place"),
            )

        elif self.animation_phase == "moving_to_place":
            self.status_label.setText("Status: Object Released")
            self.holding_object = False
            self.object_pos = list(self.arm.end_effector(self.joint_angles))
            self.redraw()

            self.animation_phase = None
            self.run_button.setEnabled(True)
            self.status_label.setText("Status: Sequence Complete")


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = ArmWindow()
    win.resize(1000, 680)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()