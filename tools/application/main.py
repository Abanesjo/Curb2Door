from PySide6.QtWidgets import QApplication
from mainwindow import MainWindow
import sys

app = QApplication(sys.argv)
w = MainWindow(app)
w.show()
app.exec()