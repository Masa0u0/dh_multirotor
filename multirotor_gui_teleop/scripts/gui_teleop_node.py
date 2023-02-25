#!/usr/bin/env python3

import os.path as osp
import sys
import signal
import rospy
from PyQt5.QtWidgets import QApplication

from multirotor_gui_teleop import GuiTeleopWidget

if __name__ == '__main__':
    node_name = osp.splitext(osp.basename(__file__))[0]
    rospy.init_node(node_name)

    app = QApplication(sys.argv)

    setup_assistant = GuiTeleopWidget()
    setup_assistant.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    sys.exit(app.exec())
