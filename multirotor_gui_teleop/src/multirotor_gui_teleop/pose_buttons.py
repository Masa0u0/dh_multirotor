from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .gui_teleop import GuiTeleopWidget

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *


class PoseButtonsWidget(QWidget):
    
    BUTTON_HEIGHT = 30
    
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
        pass
