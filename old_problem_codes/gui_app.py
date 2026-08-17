"""
Starter GUI for the pick-and-place exercise.

Run with:  python gui_app.py

What's already wired up:
  - A PyQtGraph plot that draws the arm
  - Sliders that let you manually move each joint (a working reference for
    how the redraw path works)
  - The pick bin (green circle) and place target (blue square) marked on
    the plot

What you need to build (see on_run_pick_and_place below):
  - The "Run Pick & Place" button: move the end effector to PICK_POS,
    "grab" the object, move to PLACE_POS, "release" it.
  - Do this WITHOUT freezing the window -- the sliders and the window
    itself should stay usable while the arm is moving. (Hint: look at how
    moving a slider updates the drawing, and think about what would happen
    if you used time.sleep() in a loop inside the button handler instead.)
  - A live label showing the current end-effector (x, y) -- there's
    already a label for this, it just needs to be kept up to date.

Feel free to restructure this file if you'd approach it differently --
this is scaffolding, not a required structure.
"""

import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

from arm_model import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]
PICK_POS = (4.0, 2.0)
PLACE_POS = (-3.0, 3.0)


class ArmWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pick & Place Exercise")
        self.arm = PlanarArm(LINK_LENGTHS)
        self.joint_angles = [0.3, -0.5, -0.3]

        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        self.setCentralWidget(central)

        # --- plot ---
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setXRange(-8, 8)
        self.plot_widget.setYRange(-8, 8)
        self.plot_widget.setAspectLocked(True)
        layout.addWidget(self.plot_widget, stretch=3)

        self.arm_curve = self.plot_widget.plot(pen=pg.mkPen(width=4))
        self.joint_scatter = pg.ScatterPlotItem(size=10, brush="r")
        self.plot_widget.addItem(self.joint_scatter)

        self.plot_widget.plot(
            [PICK_POS[0]], [PICK_POS[1]], pen=None,
            symbol="o", symbolBrush="g", symbolSize=18, name="pick bin",
        )
        self.plot_widget.plot(
            [PLACE_POS[0]], [PLACE_POS[1]], pen=None,
            symbol="s", symbolBrush="b", symbolSize=18, name="place target",
        )

        self.object_scatter = pg.ScatterPlotItem(size=14, brush="y")
        self.plot_widget.addItem(self.object_scatter)
        self.object_pos = list(PICK_POS)
        self.holding_object = False

        # --- controls ---
        controls = QtWidgets.QVBoxLayout()
        layout.addLayout(controls, stretch=1)

        self.sliders = []
        for i in range(self.arm.n_joints):
            controls.addWidget(QtWidgets.QLabel(f"Joint {i}"))
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(int(np.degrees(self.joint_angles[i])))
            slider.valueChanged.connect(self.on_slider_changed)
            controls.addWidget(slider)
            self.sliders.append(slider)

        self.ee_label = QtWidgets.QLabel("End effector: (?, ?)")
        controls.addWidget(self.ee_label)

        self.status_label = QtWidgets.QLabel("Status: idle")
        controls.addWidget(self.status_label)

        # TODO: add a hover effect to this button (e.g. a color change on
        # mouseover) and keep self.status_label updated as the arm's state
        # changes (idle / moving / done) -- see task 4 in the README.
        self.run_button = QtWidgets.QPushButton("Run Pick && Place")
        self.run_button.clicked.connect(self.on_run_pick_and_place)
        controls.addWidget(self.run_button)

        controls.addStretch()

        self.redraw()

    def on_slider_changed(self):
        self.joint_angles = [np.radians(s.value()) for s in self.sliders]
        self.redraw()

    def redraw(self):
        points = self.arm.forward_kinematics(self.joint_angles)
        xs, ys = zip(*points)
        self.arm_curve.setData(xs, ys)
        self.joint_scatter.setData(xs, ys)

        if self.holding_object:
            self.object_pos = list(self.arm.end_effector(self.joint_angles))
        self.object_scatter.setData([self.object_pos[0]], [self.object_pos[1]])

        ee = self.arm.end_effector(self.joint_angles)
        self.ee_label.setText(f"End effector: ({ee[0]:.2f}, {ee[1]:.2f})")

    def on_run_pick_and_place(self):
        # TODO: implement this.
        #
        # Rough shape of what's needed:
        #   1. Compute joint angles to reach PICK_POS (self.arm.inverse_kinematics)
        #   2. Animate the arm moving there smoothly (don't just jump)
        #   3. Mark the object as picked up (self.holding_object = True)
        #   4. Compute joint angles to reach PLACE_POS, animate there
        #   5. Release the object (self.holding_object = False)
        #
        # The window must stay responsive throughout -- moving a slider or
        # closing the window should still work while this is running.
        pass


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = ArmWindow()
    win.resize(900, 600)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
