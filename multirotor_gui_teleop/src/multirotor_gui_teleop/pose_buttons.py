from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .gui_teleop import GuiTeleopWidget

import rospy
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class PoseButtonsWidget(QWidget):

    BUTTON_HEIGHT = 30
    INTERVAL = 0.1

    def __init__(self, main: GuiTeleopWidget) -> None:
        super().__init__()
        self._main = main

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        self.random_button = QPushButton("Randomize")
        self.random_button.setFixedHeight(self.BUTTON_HEIGHT)
        self._rows.addWidget(self.random_button)

        self.center_button = QPushButton("Center")
        self.center_button.setFixedHeight(self.BUTTON_HEIGHT)
        self._rows.addWidget(self.center_button)

    def define_connections(self) -> None:
        self.random_button.clicked.connect(self._random_event)
        self.center_button.clicked.connect(self._center_event)

    @pyqtSlot()
    def _random_event(self) -> None:
        """ 全ての関節角をランダム値に設定する． """
        self.setEnabled(False)

        # 一気に指令すると反映されないので，間隔を開けながら指令する
        for joint_cmd in self._main.commanders.joint_cmds:
            joint_cmd.set_random_value()
            rospy.sleep(self.INTERVAL)

        self.setEnabled(True)

    @pyqtSlot()
    def _center_event(self) -> None:
        """ 全ての関節角を中央の値に設定する． """
        self.setEnabled(False)

        for joint_cmd in self._main.commanders.joint_cmds:
            joint_cmd.set_center_value()
            rospy.sleep(self.INTERVAL)

        self.setEnabled(True)
