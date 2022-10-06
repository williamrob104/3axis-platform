import ctypes

import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import QApplication

import custom_widgets


ser = serial.Serial()

ports = serial.tools.list_ports.grep("STM Serial")
port = next(ports, None)
if port:
    ser.port = port.name
    ser.open()


myappid = "mycompany.myproduct.subproduct.version"  # arbitrary string
ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(myappid)

app = QApplication([])
app.setApplicationName("Eddy current scanning platform")
app.setWindowIcon
app.setStyle("fusion")

widget = custom_widgets.MainWidget(ser)
widget.show()

app.exec()
