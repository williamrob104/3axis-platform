import struct

import numpy as np
import serial.serialutil
import serial.tools.list_ports
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import *
from serial import Serial


class MainWidget(QWidget):
    def __init__(self, serial: Serial, parent=None):
        super().__init__(parent)

        layout = QHBoxLayout()
        layout.addWidget(SideBarWidget(serial))

        self.setLayout(layout)


class MatplotlibFigureWidget(QWidget):
    def __init__(self, parent=None, include_nav_toolbar=True, **fig_kwargs):
        super().__init__(parent)

        layout = QVBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)

        self.fig = Figure(**fig_kwargs)
        bgcolor = self.palette().color(self.backgroundRole())
        self.fig.patch.set_facecolor(
            [bgcolor.red() / 255, bgcolor.green() / 255, bgcolor.blue() / 255]
        )
        canvas = FigureCanvasQTAgg(self.fig)

        if include_nav_toolbar:
            toolbar = NavigationToolbar2QT(canvas, self)
            layout.addWidget(toolbar)

        layout.addWidget(canvas)

        self.setLayout(layout)

    @property
    def figure(self) -> Figure:
        return self.fig


class DataDisplayWidget(MatplotlibFigureWidget):
    def __init__(self, parent=None, figsize=(3, 2)):
        super().__init__(
            parent, include_nav_toolbar=True, tight_layout=True, figsize=figsize
        )

        ax = self.figure.subplots()
        ax.set_xlabel("Time (ms)", fontsize=8)
        ax.set_ylabel("Voltage (V)", fontsize=8)
        ax.tick_params(axis="both", labelsize=8)
        self.ax = ax

        (self.line,) = ax.plot([], [])

    def displayData(self, x, y):
        self.line.set_data(x, y)
        self.ax.relim()
        self.ax.autoscale_view(True, True, True)
        self.fig.canvas.draw()


class SideBarWidget(QWidget):
    def __init__(self, serial: Serial, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout()
        layout.setSizeConstraint(QLayout.SizeConstraint.SetFixedSize)

        data_display_widget = DataDisplayWidget()

        layout.addWidget(QLabel("<b>Serial Port<b>"))
        layout.addWidget(PortConnectWidget(serial))

        layout.addWidget(QLabel("<b>Manual Control<b>"))
        layout.addWidget(ManualControlWidget(serial, data_display_widget.displayData))

        layout.addWidget(QLabel("<b>Time Response<b>"))
        layout.addWidget(data_display_widget)

        self.setLayout(layout)


class PortConnectWidget(QWidget):
    def __init__(self, serial: Serial, parent=None):
        super().__init__(parent)
        self.serial = serial

        self.port_selection_widget = PortSelectionWidget()
        self.port_selection_widget.setPortName(self.serial.port)

        button = QPushButton("Connect")
        button.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        button.clicked.connect(self.onButtonClicked)

        layout = QVBoxLayout()
        layout.addWidget(self.port_selection_widget)
        layout.addWidget(button, 0, Qt.AlignmentFlag.AlignRight)

        self.setLayout(layout)

    def onButtonClicked(self):
        port = self.port_selection_widget.getPortName()
        if self.serial.isOpen() and self.serial.port == port:
            return

        self.serial.close()
        self.serial.port = port
        self.serial.write_timeout = 0.1
        try:
            self.serial.open()
            self.serial.write(b"test\n")
        except serial.serialutil.SerialTimeoutException:
            self.serial.close()
            display_error_message("Cannot connect to port.")
        except serial.serialutil.SerialException as e:
            self.serial.close()
            display_error_message(str(e))


class PortSelectionWidget(QComboBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.refreshItems()

    def showPopup(self) -> None:
        self.refreshItems()
        return super().showPopup()

    def refreshItems(self):
        self.clear()
        for port in serial.tools.list_ports.comports():
            self.addItem(port.description, port.name)

    def getPortName(self):
        return self.currentData()

    def setPortName(self, port_name):
        idx = self.findData(port_name)
        if idx != -1:
            self.setCurrentIndex(idx)


class ManualControlWidget(QWidget):
    def __init__(self, serial: Serial, data_display_func, parent=None):
        super().__init__(parent)

        layout = QFormLayout()
        layout.setHorizontalSpacing(30)
        layout.setVerticalSpacing(15)
        jog_position_widget = JogPositionWidget(serial)
        layout.addRow(QLabel("Jog Position"), jog_position_widget)
        layout.addRow(
            QLabel("Jog Distance"),
            JogDistanceWidget(jog_position_widget.setJogDistance),
        )
        layout.addRow(QLabel("Probe"), ProbeControlWidget(serial, data_display_func))
        layout.addRow(QLabel("Send G-code"), SendGcodeWidget(serial))
        self.setLayout(layout)


class JogPositionWidget(QWidget):
    def __init__(self, serial: Serial, parent=None):
        super().__init__(parent)
        self.serial = serial

        xy_buttons = QGridLayout()
        button = QToolButton()
        button.setIcon(load_icon("chevron-up.png"))
        button.clicked.connect(lambda: self.onMoveButtonClicked("Y"))
        xy_buttons.addWidget(button, 0, 1)
        button = QToolButton()
        button.setIcon(load_icon("chevron-left.png"))
        button.clicked.connect(lambda: self.onMoveButtonClicked("X-"))
        xy_buttons.addWidget(button, 1, 0)
        button = QToolButton()
        button.setIcon(load_icon("home.png"))
        button.clicked.connect(lambda: self.onHomeButtonClicked("X Y"))
        xy_buttons.addWidget(button, 1, 1)
        button = QToolButton()
        button.setIcon(load_icon("chevron-right.png"))
        button.clicked.connect(lambda: self.onMoveButtonClicked("X"))
        xy_buttons.addWidget(button, 1, 2)
        button = QToolButton()
        button.setIcon(load_icon("chevron-down.png"))
        button.clicked.connect(lambda: self.onMoveButtonClicked("Y-"))
        xy_buttons.addWidget(button, 2, 1)

        z_buttons = QGridLayout()
        button = QToolButton()
        button.setIcon(load_icon("chevron-up.png"))
        button.clicked.connect(lambda: self.onMoveButtonClicked("Z"))
        z_buttons.addWidget(button, 0, 0)
        button = QToolButton()
        button.setIcon(load_icon("home.png"))
        button.clicked.connect(lambda: self.onHomeButtonClicked("Z"))
        z_buttons.addWidget(button, 1, 0)
        button = QToolButton()
        button.setIcon(load_icon("chevron-down.png"))
        button.clicked.connect(lambda: self.onMoveButtonClicked("Z-"))
        z_buttons.addWidget(button, 2, 0)

        layout = QGridLayout()
        layout.setSizeConstraint(QLayout.SizeConstraint.SetFixedSize)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(10)

        layout.addWidget(QLabel("X/Y"), 0, 0, Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(QLabel("Z"), 0, 1, Qt.AlignmentFlag.AlignCenter)
        layout.addLayout(xy_buttons, 1, 0, Qt.AlignmentFlag.AlignCenter)
        layout.addLayout(z_buttons, 1, 1, Qt.AlignmentFlag.AlignCenter)

        self.setLayout(layout)

    def setJogDistance(self, jog_distance):
        self.jog_distance = jog_distance

    def onMoveButtonClicked(self, dir):
        if self.serial.isOpen():
            gcode = f"G91\nG0 {dir}{self.jog_distance:.3f}\n"
            self.serial.write(gcode.encode())

    def onHomeButtonClicked(self, dir):
        if self.serial.isOpen():
            gcode = f"G28 {dir}\n"
            self.serial.write(gcode.encode())


class JogDistanceWidget(QWidget):
    def __init__(self, set_jog_distance_func, parent=None):
        super().__init__(parent)
        self.set_jog_distance_func = set_jog_distance_func

        layout = QHBoxLayout()
        layout.setSizeConstraint(QLayout.SizeConstraint.SetFixedSize)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        dists = ["0.1", "1", "10", "50"]
        self.buttons = []
        for i, dist in enumerate(dists):
            button = QToolButton()
            button.setText(dist)
            button.setCheckable(True)
            button.clicked.connect(lambda _, i=i: self.onButtonClicked(i))
            layout.addWidget(button)
            self.buttons.append(button)

        self.setLayout(layout)

        self.onButtonClicked(0)

    def onButtonClicked(self, i):
        for button in self.buttons:
            button.setChecked(False)
        self.buttons[i].setChecked(True)
        self.set_jog_distance_func(float(self.buttons[i].text()))


class ProbeControlWidget(QWidget):
    def __init__(self, serial: Serial, data_display_func, parent=None):
        super().__init__(parent)
        self.serial = serial
        self.data_display_func = data_display_func

        layout = QHBoxLayout()
        layout.setSizeConstraint(QLayout.SizeConstraint.SetFixedSize)
        layout.setContentsMargins(0, 0, 0, 0)

        button = QToolButton()
        button.setText("Pulse")
        button.clicked.connect(self.onPulseButtonClicked)
        layout.addWidget(button)

        button = QToolButton()
        button.setText("Reset")
        button.clicked.connect(self.onResetButtonClicked)
        layout.addWidget(button)

        self.setLayout(layout)

    def onPulseButtonClicked(self):
        if self.serial.isOpen():
            self.serial.write(b"M12\n")

            self.serial.timeout = None

            token = b"period"
            self.serial.read_until(token)

            token = b"data"
            r = self.serial.read_until(token)
            data = r[: -len(token)]
            (sampling_period_ms,) = struct.unpack("f", data)

            token = b"OK\n"
            r = self.serial.read_until(token)
            data = r[: -len(token)]
            samples = struct.unpack(f"{len(data)//2}h", data)

            t = np.arange(0, len(samples)) * sampling_period_ms
            y = np.array(samples) / 4095 * 3.3
            self.data_display_func(t, y)

    def onResetButtonClicked(self):
        if self.serial.isOpen():
            self.serial.write(b"M13\n")


class SendGcodeWidget(QLineEdit):
    def __init__(self, serial: Serial, parent=None):
        super().__init__(parent)
        self.serial = serial

        self.returnPressed.connect(self.onReturnPressed)

    def onReturnPressed(self):
        if self.serial.isOpen():
            gcode = self.text() + "\n"
            self.clear()
            self.serial.write(gcode.encode())


def display_error_message(text):
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Icon.Critical)
    msg.setText(text)
    msg.setWindowTitle("Error")
    msg.exec()


def load_icon(filename) -> QIcon:
    return QIcon("./icons/" + filename)
