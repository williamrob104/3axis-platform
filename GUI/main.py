import sys

import serial.tools.list_ports
from PyQt6.QtWidgets import QApplication, QMainWindow
from serial import Serial

import custom_widgets

ser = Serial()

ports = serial.tools.list_ports.grep("STM Serial")
port = next(ports, None)
if port:
    ser.port = port.name
    ser.open()

app = QApplication(sys.argv)
app.setApplicationName("Eddy current scanning platform")
app.setStyle("fusion")

widget = custom_widgets.SideBarWidget(ser)

window = QMainWindow()
window.setCentralWidget(widget)
# window.showMaximized()
window.show()

# Start the event loop.
sys.exit(app.exec())
