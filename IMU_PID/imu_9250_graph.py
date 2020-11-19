'''
https://www.learnpyqt.com/courses/graphics-plotting/plotting-pyqtgraph/
https://www.cnblogs.com/linyfeng/p/12239856.html
'''
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import pyqtSignal
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
from random import randint
import serial
import signal

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.serial = SerialThread()
        self.serial.trigger.connect(self.update_plot_data)
        self.serial.start()

        self.graphWidget = pg.PlotWidget()
        self.graphWidget.setYRange(-30, 30)
        self.setCentralWidget(self.graphWidget)

        self.x = list(range(100))  # 100 time points
        self.y1 = [0 for _ in range(100)]  # 100 data points
        self.y2 = [0 for _ in range(100)]  # 100 data points

        self.graphWidget.setBackground('w')

        pen1 = pg.mkPen(color=(255, 0, 0))
        self.data_line1 =  self.graphWidget.plot(self.x, self.y1, pen=pen1)
        
        pen2 = pg.mkPen(color=(0, 255, 0))
        self.data_line2 =  self.graphWidget.plot(self.x, self.y2, pen=pen2)
        # self.timer = QtCore.QTimer()
        # self.timer.setInterval(50)
        # self.timer.timeout.connect(self.update_plot_data)
        # self.timer.start()

    def update_plot_data(self, error_1, error_2):

        self.x = self.x[1:]  # Remove the first y element.
        self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last.

        self.y1 = self.y1[1:]  # Remove the first 
        self.y1.append(error_1)  # Add a new random value.
        
        self.y2 = self.y2[1:]  # Remove the first 
        self.y2.append(error_2)  # Add a new random value.

        self.data_line1.setData(self.x, self.y1)  # Update the data.
        self.data_line2.setData(self.x, self.y2)  # Update the data.

class SerialThread(QtCore.QThread):
    trigger = pyqtSignal(float, float)

    def __init__(self):
        super(SerialThread, self).__init__()
        port = '/dev/ttyACM0'
        self.arduino = serial.Serial(port, 115200, timeout=1)

        if not self.arduino.is_open:
            self.arduino.open()
        for i in range(5):
            self.arduino.readline()
    
    def run(self):
        while self.arduino.is_open:
            raw_data = self.arduino.readline()
            # print(raw_data)
            try:
                data = [float(val) for val in raw_data.split()]
                print(data)
                self.trigger.emit(data[0], data[1])
            except Exception:
                print('exception occur')
            # if len(data) == 2:

signal.signal(signal.SIGINT, signal.SIG_DFL)
app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())
