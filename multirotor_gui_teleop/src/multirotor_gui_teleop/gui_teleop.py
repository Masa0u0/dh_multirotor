import os.path as osp
import rospy
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

        self.commanders = CommandersWidget(self)
        self._rows.addWidget(self.commanders)

        self.pose_buttons = PoseButtonsWidget(self)
        self._rows.addWidget(self.pose_buttons)

        self.define_connections()
        
        # 接続が完了するまで少し待ってから全ての関節値を発行
        rospy.sleep(0.5)
        self.commanders.publish()
    
    def define_connections(self) -> None:
        self.commanders.define_connections()
        self.pose_buttons.define_connections()
