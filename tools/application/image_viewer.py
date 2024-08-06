from PySide6.QtCore import QTimer, Signal, Slot, Qt
from PySide6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PySide6.QtGui import QPixmap, QImage
from sensor_msgs.msg import CompressedImage
import rospy
import cv2
import numpy as np

class ImageViewer(QLabel):
    def __init__(self, parent=None):
        super(ImageViewer, self).__init__(parent)
        self.setScaledContents(True)
        self.setAlignment(Qt.AlignCenter)

    @Slot(np.ndarray)
    def setImage(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_img = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img)
        self.setPixmap(pixmap)

class MainWindow(QMainWindow):
    append_text_signal = Signal(str, object)
    new_image_signal = Signal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.image_viewer = ImageViewer(self)
        self.image_topic = "/front_camera_image/compressed"

        # Layout to hold the image viewer
        layout = QVBoxLayout()
        layout.addWidget(self.image_viewer)
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.new_image_signal.connect(self.image_viewer.setImage)

        # Initialize ROS node
        rospy.init_node('image_viewer', anonymous=True)
        rospy.Subscriber(self.image_topic, CompressedImage, self.image_callback)

    def image_callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.new_image_signal.emit(cv_image)

    def closeEvent(self, event):
        # Perform any cleanup before the window closes
        rospy.signal_shutdown('Window closed')
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication([])
    main_win = MainWindow()
    main_win.show()
    app.exec()
