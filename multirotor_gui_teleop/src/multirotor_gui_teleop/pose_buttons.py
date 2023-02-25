from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .gui_teleop import GuiTeleopWidget

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class PoseButtonsWidget(QWidget):
    
    def __init__(self, main: GuiTeleopWidget) -> None:
        super().__init__()
        self._main = main

        # TODO
