import os.path as osp
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

from dh_rqt_tools.path import get_proj_path

from .commanders import CommandersWidget
from .pose_buttons import PoseButtonsWidget


class GuiTeleopWidget(QWidget):

    def __init__(self) -> None:
        super().__init__()

        proj_path = get_proj_path()
        icon_path = osp.join(proj_path, "resources/icon.png")
        self.setWindowIcon(QIcon(icon_path))
        self.setWindowTitle("Multirotor GUI Teleop")

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        self.joint_positions = CommandersWidget(self)
        self._rows.addWidget(self.joint_positions)

        self.pose_buttons = PoseButtonsWidget(self)
        self._rows.addWidget(self.pose_buttons)
