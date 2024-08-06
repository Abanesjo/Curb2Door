from PySide6.QtWidgets import QApplication
from mainwindow import MainWindow
import sys
import os
import qdarktheme

if sys.platform == "linux":
    # Fixes "undefined symbol: wl_proxy_marshal_flags": https://bugreports.qt.io/browse/QTBUG-114635}}
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

app = QApplication(sys.argv)
app.setStyleSheet(qdarktheme.load_stylesheet())
stylesheet = qdarktheme.load_stylesheet('light')
QApplication.instance().setStyleSheet(stylesheet)

w = MainWindow(app)
w.show()
app.exec()