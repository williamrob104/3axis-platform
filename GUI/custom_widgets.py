from PyQt6.QtCore import Qt
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import *
import numpy as np
from serial import Serial
from serial.tools import list_ports
from serial.serialutil import SerialException, SerialTimeoutException
import struct


def figure_widget(use_navigation_toolbar=True, **fig_kwargs):
    from matplotlib.backends.backend_qtagg import (
        FigureCanvasQTAgg,
        NavigationToolbar2QT,
    )
    from matplotlib.figure import Figure

    fig = Figure(**fig_kwargs)
    canvas = FigureCanvasQTAgg(fig)

    if use_navigation_toolbar:
        layout = QVBoxLayout()
        layout.setSpacing(0)  # spacing between navigation toolbar and figure is 0
        layout.setContentsMargins(0, 0, 0, 0)  # margin around this widget 0
        widget = QWidget()
        widget.setLayout(layout)
        toolbar = NavigationToolbar2QT(canvas, widget)
        layout.addWidget(toolbar)
        layout.addWidget(canvas)
        return fig, widget
    else:
        return fig, canvas


class MainWidget(QWidget):
    def __init__(self, serial: Serial, parent=None):
        super().__init__(parent)

        fig, widget = figure_widget(False, tight_layout=True)
        ax1 = fig.add_subplot(111)
        t = np.linspace(0, 4e-3)
        tau = 0.36e-3
        y = 4094 * (1 - np.exp(-t / tau))
        y = y.astype("int")
        ax1.plot(t, y)

        layout = QHBoxLayout()
        layout.addWidget(widget)
        layout.addWidget(SideBarWidget(serial))

        self.setLayout(layout)


class SideBarWidget(QWidget):
    def __init__(self, serial: Serial, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout()

        layout.addWidget(QLabel("<b>Serial Port<b>"))
        layout.addWidget(PortConnectWidget(serial))

        layout.addWidget(QLabel("<b>Manual Control<b>"))
        layout.addWidget(ManualControlWidget(serial))

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
        except SerialTimeoutException:
            self.serial.close()
            displayErrorMessage("Cannot connect to port.")
        except SerialException as e:
            displayErrorMessage(str(e))


class PortSelectionWidget(QComboBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.refreshItems()

    def showPopup(self) -> None:
        self.refreshItems()
        return super().showPopup()

    def refreshItems(self):
        self.clear()
        for port in list_ports.comports():
            self.addItem(port.description, port.name)

    def getPortName(self):
        return self.currentData()

    def setPortName(self, port_name):
        idx = self.findData(port_name)
        if idx != -1:
            self.setCurrentIndex(idx)


class ManualControlWidget(QWidget):
    def __init__(self, serial: Serial, parent=None):
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
        layout.addRow(QLabel("Probe"), ProbeControlWidget(serial))
        layout.addRow(QLabel("Send G-code"), SendGcodeWidget(serial))
        self.setLayout(layout)


class JogPositionWidget(QWidget):
    def __init__(self, serial: Serial, parent=None):
        super().__init__(parent)
        self.serial = serial

        xy_buttons = QGridLayout()
        button = QToolButton()
        button.setIcon(loadIcon("chevron-up.png"))
        button.clicked.connect(lambda: self.onMoveButtonClicked("Y"))
        xy_buttons.addWidget(button, 0, 1)
        button = QToolButton()
        button.setIcon(loadIcon("chevron-left.png"))
        button.clicked.connect(lambda: self.onMoveButtonClicked("X-"))
        xy_buttons.addWidget(button, 1, 0)
        button = QToolButton()
        button.setIcon(loadIcon("home.png"))
        button.clicked.connect(lambda: self.onHomeButtonClicked("X Y"))
        xy_buttons.addWidget(button, 1, 1)
        button = QToolButton()
        button.setIcon(loadIcon("chevron-right.png"))
        button.clicked.connect(lambda: self.onMoveButtonClicked("X"))
        xy_buttons.addWidget(button, 1, 2)
        button = QToolButton()
        button.setIcon(loadIcon("chevron-down.png"))
        button.clicked.connect(lambda: self.onMoveButtonClicked("Y-"))
        xy_buttons.addWidget(button, 2, 1)

        z_buttons = QGridLayout()
        button = QToolButton()
        button.setIcon(loadIcon("chevron-up.png"))
        button.clicked.connect(lambda: self.onMoveButtonClicked("Z"))
        z_buttons.addWidget(button, 0, 0)
        button = QToolButton()
        button.setIcon(loadIcon("home.png"))
        button.clicked.connect(lambda: self.onHomeButtonClicked("Z"))
        z_buttons.addWidget(button, 1, 0)
        button = QToolButton()
        button.setIcon(loadIcon("chevron-down.png"))
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
    def __init__(self, serial: Serial, parent=None):
        super().__init__(parent)
        self.serial = serial

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

            print(
                sampling_period_ms,
                sampling_period_ms * len(samples),
                samples[0],
                samples[-1],
            )

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


def displayErrorMessage(text):
    msg = QMessageBox()
    msg.setIcon(QMessageBox.Icon.Critical)
    msg.setText(text)
    msg.setWindowTitle("Error")
    msg.exec()


def loadIcon(filename) -> QIcon:
    return QIcon("./icons/" + filename)
