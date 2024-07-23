from PySide6.QtCore import QTimer, Signal, Slot, Qt
from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide6.QtGui import QPixmap, QImage
from ui_mainwindow import Ui_MainWindow
from topic_monitor import TopicMonitor

import paramiko
import subprocess
import threading
import os
import re
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage

def clean_ansi_sequences(text):
    ansi_escape = re.compile(r'''
        \x1B[@-_][0-?]*[ -/]*[@-~]  # ANSI escape sequences
    ''', re.VERBOSE)
    return ansi_escape.sub('', text)

class MainWindow(QMainWindow, Ui_MainWindow):
    new_image_signal = Signal(np.ndarray)
    append_text_signal = Signal(str, object)
    dimension_signal = Signal(int, int)

    def __init__(self, app):
        super().__init__()
        self.setupUi(self)
        self.app = app

        self.address = ""
        self.user = ""
        self.password = ""

        self.SSH = paramiko.SSHClient()
        self.SSH.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        self.button_connect.clicked.connect(self.ssh_connect)
        self.button_disconnect.clicked.connect(self.ssh_disconnect)
        self.button_build_source.clicked.connect(self.build_and_source)
        self.button_chmod.clicked.connect(self.chmod)
        self.button_start.clicked.connect(self.start_sensors)
        self.button_stop.clicked.connect(self.stop_sensors)
        self.button_preview.clicked.connect(self.preview)
        self.combo_image_topic.currentIndexChanged.connect(self.update_preview)
        self.button_end_preview.clicked.connect(self.end_preview)
        self.button_begin_recording(self.begin_recording)
        self.button_end_recording(self.end_recording)
        self.button_monitor.clicked.connect(self.monitor)

        self.workspace_path = ""
        self.setup = ""
        self.update_workspace_path()
        
        self.new_image_signal.connect(self.setImage)
        self.append_text_signal.connect(self.append_text)
        self.dimension_signal.connect(self.update_image_shape)

        self.rostopic_processes = {}

        # Set initial black image
        self.set_black_image()

    def __del__(self):
        print("Disconnecting from the robot")
        self.SSH.close()

    def closeEvent(self, event):
    # Perform any cleanup before the window closes
        self.ssh_disconnect()  # Close SSH connections if open

        # Close any active ROS subscribers
        if hasattr(self, 'image_subscriber'):
            self.image_subscriber.unregister()
        
        # Stop any active rostopic processes
        sessions_to_close = list(self.rostopic_processes.values())
        for session in sessions_to_close:
            session.close()
        self.rostopic_processes.clear()

        # Shutdown ROS node
        if rospy.core.is_initialized():
            rospy.signal_shutdown('GUI closing')

        super().closeEvent(event)

    def append_text(self, text, text_widget):
        text_widget.append(text)

    def set_black_image(self):
        height, width = self.label_image_preview.size().height(), self.label_image_preview.size().width()
        black_image = np.zeros((height, width, 3), dtype=np.uint8)
        q_img = QImage(black_image.data, width, height, 3 * width, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img)
        self.label_image_preview.setPixmap(pixmap)

    def execute_local(self, command):
        self.text_log.append(f"(Local) {command}")
        def run_command():
            try:
                process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

                while True:
                    output = process.stdout.readline()
                    if output == '' and process.poll() is not None:
                        break
                    if output:
                        print(output.strip())

                process.wait()

                if process.returncode != 0:
                    raise subprocess.CalledProcessError(process.returncode, command)

            except subprocess.CalledProcessError as e:
                print(f"Error: {e}")

        thread = threading.Thread(target=run_command)
        thread.daemon = True
        thread.start()

    def execute_and_print(self, command, text_widget, command_id=None):
        def run_command():
            ssh = self.SSH
            try:
                session = ssh.get_transport().open_session()
                session.get_pty()
                session.exec_command(command)

                # Store the session to allow it to be closed later
                if command_id:
                    self.rostopic_processes[command_id] = session

                while True:
                    line = session.recv(1024).decode('utf-8')
                    if not line:
                        break
                    cleaned_line = clean_ansi_sequences(line.strip())
                    self.append_text_signal.emit(cleaned_line, text_widget)

                while True:
                    line = session.recv_stderr(1024).decode('utf-8')
                    if not line:
                        break
                    cleaned_line = clean_ansi_sequences(line.strip())
                    self.append_text_signal.emit(cleaned_line, text_widget)

            except Exception as e:
                error_message = f"Error executing command: {str(e)}"
                self.append_text_signal.emit(error_message, text_widget)

            self.append_text_signal.emit("------------------------------------------------", text_widget)

        thread = threading.Thread(target=run_command)
        thread.daemon = True        
        thread.start()

    def execute_sudo_command(self, command):
        ssh = self.SSH
        session = ssh.get_transport().open_session()
        session.set_combine_stderr(True)
        session.get_pty()
        session.exec_command(f"sudo -S -p '' {command}")
        self.text_log.append(f"sudo {command}")

        stdin = session.makefile('wb', -1)
        stdout = session.makefile('rb', -1)

        stdin.write(self.password + '\n')
        stdin.flush()

        stdout_data = stdout.read()

        stdin.close()
        stdout.close()
        session.close()

        return stdout_data.decode()

    def update_workspace_path(self):
        self.workspace_path = self.line_workspace_path.text()
        self.setup = f"cd {self.workspace_path} && source devel/setup.bash"

    def ssh_connect(self):
        self.address = self.line_address.text()
        self.user = self.line_user.text()
        self.password = self.line_password.text()
        self.line_connection_status.setText('Connecting...')
        self.text_log.append(f"Connecting to {self.user}@{self.address}...")
        
        #Set ROS_MASTER_URI
        QApplication.processEvents()

        QTimer.singleShot(100, lambda: self._attempt_ssh_connection(self.address, self.user, self.password))

    def _attempt_ssh_connection(self, address, user, password):
        try:
            self.SSH.connect(address, username=user, password=password)
            self.line_connection_status.setText('Connected')
            self.text_log.append("Connected.")
            os.environ['ROS_MASTER_URI'] = f"http://{self.address}:11311"
            self.text_log.append(f"ROS_MASTER_URI: {os.environ['ROS_MASTER_URI']}")
        except Exception as e:
            self.line_connection_status.setText('Disconnected')
            self.text_log.append(f"Could not connect: {str(e)}")

    def ssh_disconnect(self):
        if self.SSH:
            self.SSH.close()
            self.text_log.append("Disconnecting from Remote.")
            self.line_connection_status.setText('Disconnected')
        QApplication.quit()

    def build_and_source(self):
        self.update_workspace_path()
        self.text_log.append(f"Workspace Path: {self.workspace_path}")
        self.execute_and_print(f"cd {self.workspace_path} && catkin build && source devel/setup.bash", self.text_log)

    def chmod(self):
        self.text_log.append("-------Granting Port Permissions-----")
        command = """echo SUBSYSTEM=='"usb"', ATTR{idVendor}=='"2e1a"', SYMLINK+='"insta"' | sudo tee /etc/udev/rules.d/98-insta.rules"""
        self.execute_sudo_command(command)
        self.execute_sudo_command("udevadm trigger")
        self.execute_sudo_command("chmod 777 /dev/insta")
        self.text_log.append("--------Port Permission Status---------")
        self.execute_and_print("ls -lR /dev | grep insta", self.text_log)

    def start_sensors(self):
        self.update_workspace_path()
        self.text_log.append("Starting Sensors...")
        self.SSH.exec_command(f"{self.setup} && roslaunch curb2door bringup.launch")
        self.execute_and_print(f'{self.setup} && echo "Live Topics:\n" && rostopic list', self.text_log)

    def stop_sensors(self):
        self.update_workspace_path()
        self.text_log.append("Stoping Sensors...")
        # self.SSH.exec_command(f"{self.setup} && rosnode kill /live_processing /raw_output")
        self.SSH.exec_command("pkill -f ros")
        self.set_black_image()

    def preview(self):
        self.text_log.append("Opening Preview (Local)...")
        self.update_preview()
    
    def update_preview(self):
        if not rospy.core.is_initialized():
            rospy.init_node("gui", anonymous=True, disable_signals=True)
        try:
            self.image_subscriber.unregister()
        except Exception as e:
            pass

        if self.combo_image_topic.currentIndex() == 0:
            image_topic = "/front_camera_image/compressed"
        elif self.combo_image_topic.currentIndex() == 1:
            image_topic = "/back_camera_image/compressed"
        else:
            print("Invalid Index")
            return
        
        self.image_subscriber = rospy.Subscriber(image_topic, CompressedImage, self.image_callback)

    def end_preview(self):
        self.text_log.append("Ending Preview (Local)...")
        if hasattr(self, 'image_subscriber'):
            self.image_subscriber.unregister()
            del self.image_subscriber
        self.set_black_image()

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.new_image_signal.emit(cv_image)
        self.dimension_signal.emit(cv_image.shape[1], cv_image.shape[0])  # Emit dimension signal

    @Slot(np.ndarray)
    def setImage(self, cv_image):
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_img = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img)
        
        scaled_pixmap = pixmap.scaled(self.label_image_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.label_image_preview.setPixmap(scaled_pixmap)

    def begin_recording(self):
        self.update_workspace_path()
        bag_path = self.line_bag_path.text()
        bag_name = self.line_bag_name.text()
        self.text_log.append(f"Recording to {bag_path}/{bag_name}")
        self.SSH.exec_command(f"{self.setup} && roslaunch curb2door record.launch bag_path:={bag_path} bag_name:={bag_name}")
        self.line_record_status.setText("Recording")
        self.line_bag_name_check.setText(bag_name)

    def end_recording(self):
        self.update_workspace_path()
        self.text_log.append("Ending Recording...")
        self.SSH.exec_command(f"{self.setup} && rosnode kill /rosbag")
        self.line_record_status.setText("Not Recording")
        self.line_bag_name_check.clear()     

    def monitor(self):
        self.text_log.append("Begin monitoring topics")

        # Stop any existing rostopic commands
        sessions_to_close = list(self.rostopic_processes.values())
        for session in sessions_to_close:
            session.close()
        self.rostopic_processes.clear()

        # Clear the text boxes
        self.text_image_freq.clear()
        self.text_lidar_freq.clear()
        self.text_imu_freq.clear()
        self.line_image_size.clear()

        # Start new monitor commands
        image_topic = self.line_image_topic.text()
        self.execute_and_print(f"{self.setup} && rostopic hz {image_topic}", self.text_image_freq, command_id="image")

        lidar_topic = self.line_lidar_topic.text()
        self.execute_and_print(f"{self.setup} && rostopic hz {lidar_topic}", self.text_lidar_freq, command_id="lidar")

        imu_topic = self.line_imu_topic.text()
        self.execute_and_print(f"{self.setup} && rostopic hz {imu_topic}", self.text_imu_freq, command_id="imu")

        # Initialize ROS node if not already initialized
        if not rospy.core.is_initialized():
            rospy.init_node("gui_monitor", anonymous=True, disable_signals=True)

        # Create the subscriber for the image topic to get the image dimensions
        if hasattr(self, 'image_subscriber'):
            self.image_subscriber.unregister()
        self.image_subscriber = rospy.Subscriber(image_topic, CompressedImage, self.image_callback)

    def update_image_shape(self, width, height):
        self.line_image_size.setText(f"({width}, {height})")

    
