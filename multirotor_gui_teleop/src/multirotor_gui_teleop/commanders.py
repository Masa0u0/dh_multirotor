from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .gui_teleop import GuiTeleopWidget

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import rospy
from urdf_parser_py.urdf import Robot
from std_msgs.msg import Float64

from dh_rqt_tools.widgets import Slider
from multirotor_msgs.msg import Command

from .utils import remap


class CommandersWidget(QScrollArea):

    def __init__(self, main: GuiTeleopWidget) -> None:
        super().__init__()
        self._main = main

        self.setWidgetResizable(True)  # この設定が必須．無いとオブジェクトが潰れてしまう．

        # QScrollAreaを使う際は，QLayoutの前にQWidgetを挟む必要がある．
        inner = QWidget()
        self.setWidget(inner)
        self._rows = QVBoxLayout()
        inner.setLayout(self._rows)

        self._drone_cmd = Command()

        # ドローンの位置姿勢
        x_min = rospy.get_param("~pose_limit/x/min")
        x_max = rospy.get_param("~pose_limit/x/max")
        assert x_min <= 0. <= x_max
        self.drone_cmd_x = Commander("multirotor/x", x_min, x_max)
        self._rows.addWidget(self.drone_cmd_x)

        y_min = rospy.get_param("~pose_limit/y/min")
        y_max = rospy.get_param("~pose_limit/y/max")
        assert y_min <= 0. <= y_max
        self.drone_cmd_y = Commander("multirotor/y", y_min, y_max)
        self._rows.addWidget(self.drone_cmd_y)

        z_min = rospy.get_param("~pose_limit/z/min")
        z_max = rospy.get_param("~pose_limit/z/max")
        assert 0. <= z_min <= z_max
        self.drone_cmd_z = Commander("multirotor/z", z_min, z_max)
        self._rows.addWidget(self.drone_cmd_z)

        yaw_min = rospy.get_param("~pose_limit/yaw/min")
        yaw_max = rospy.get_param("~pose_limit/yaw/max")
        assert yaw_min <= 0. <= yaw_max
        self.drone_cmd_yaw = Commander("multirotor/yaw", yaw_min, yaw_max)
        self._rows.addWidget(self.drone_cmd_yaw)

        # その他の可動関節
        drone_name = rospy.get_param("/drone_name")
        joint_names = rospy.get_param("/required_joint_names")
        robot = Robot.from_parameter_server("/robot_description")
        self.joint_cmds = []
        for joint_name in joint_names:
            joint = robot.joint_map[joint_name]
            commander = Commander(
                joint_name,
                joint.limit.lower,
                joint.limit.upper,
                f'/{drone_name}/{joint_name}_controller/command',
            )
            commander.update()
            self.joint_cmds.append(commander)
            self._rows.addWidget(commander)

        self._drone_cmd_pub = rospy.Publisher(
            "/multirotor_controller/command", Command, queue_size=1
        )
        
        # ダミーウィジェット
        self._add_dummy_widget()

        self.drone_cmd_x.value_changed.connect(self._publish_drone_cmd)
        self.drone_cmd_y.value_changed.connect(self._publish_drone_cmd)
        self.drone_cmd_z.value_changed.connect(self._publish_drone_cmd)
        self.drone_cmd_yaw.value_changed.connect(self._publish_drone_cmd)

        rospy.sleep(0.5)  # 接続に少し時間がかかるため，初期値を設定する前に待機する

        self.drone_cmd_x.set_value(0.)
        self.drone_cmd_y.set_value(0.)
        self.drone_cmd_z.set_value(z_min)
        self.drone_cmd_yaw.set_value(0.)

    @pyqtSlot()
    def _publish_drone_cmd(self) -> None:
        self._drone_cmd.target_position.x = self.drone_cmd_x.get_value()
        self._drone_cmd.target_position.y = self.drone_cmd_y.get_value()
        self._drone_cmd.target_position.z = self.drone_cmd_z.get_value()
        self._drone_cmd.target_yaw_angle = self.drone_cmd_yaw.get_value()

        self._drone_cmd_pub.publish(self._drone_cmd)

    def _add_dummy_widget(self) -> None:
        dummy_widget = QWidget()
        dummy_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._rows.addWidget(dummy_widget)


class Commander(QWidget):

    PSIZE = 9
    RANGE = 10000

    value_changed = pyqtSignal(float)

    def __init__(self, name: str, minimum: float, maximum: float, topic: str = None) -> None:
        super().__init__()
        self._min = minimum
        self._max = maximum
        self._topic = topic

        font = QFont("Default", self.PSIZE, QFont.Bold)

        self._rows = QVBoxLayout()
        self.setLayout(self._rows)

        self._cols = QHBoxLayout()
        self._rows.addLayout(self._cols)

        self.name = QLabel(name)
        self.name.setFont(font)
        self._cols.addWidget(self.name)

        self.value = QLineEdit("0.00")
        self.value.setAlignment(Qt.AlignRight)
        self.value.setFont(font)
        self.value.setReadOnly(True)
        self._cols.addWidget(self.value)

        self.slider = Slider(Qt.Horizontal)
        self.slider.setFont(font)
        self.slider.setRange(0, self.RANGE)
        self.slider.setValue(self.RANGE // 2)
        self._rows.addWidget(self.slider)

        if self._topic:
            self._value_pub = rospy.Publisher(topic, Float64, queue_size=1)

        self.slider.valueChanged.connect(self._on_value_changed)

    def get_value(self) -> float:
        return float(self.value.text())

    def set_value(self, value: float) -> None:
        slider_value = self._value_to_slider(value)
        self.slider.setValue(slider_value)

    @pyqtSlot()
    def _on_value_changed(self) -> None:
        value = self._slider_to_value()
        self.value.setText(f'{value:.2f}')
        self.value_changed.emit(value)

        if self._topic:
            self._value_pub.publish(Float64(data=value))

    def _slider_to_value(self) -> float:
        x = float(self.slider.value())
        return remap(x, 0., self.RANGE, self._min, self._max)

    def _value_to_slider(self, value: float) -> int:
        assert self._min <= value <= self._max
        return int(remap(value, self._min, self._max, 0., self.RANGE))
