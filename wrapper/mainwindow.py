from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from ui_mainwindow import Ui_MainWindow
from topic_monitor import TopicMonitor

import paramiko
import subprocess


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, app):
        super().__init__()
        self.setupUi(self)
        self.app = app

        self.SSH = paramiko.SSHClient()
        self.SSH.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        self.button_connect.clicked.connect(self.ssh_connect)
        self.button_disconnect.clicked.connect(self.ssh_disconnect)
        self.button_build_source.clicked.connect(self.build_and_source)
        self.button_record.clicked.connect(self.record)

        self.workspace_path = ""
        self.update_workspace_path()

    def __del__(self):
        print("Disconnecting from the robot")
        self.SSH.close()

    def update_workspace_path(self):
        self.workspace_path = self.line_workspace_path.text()
        print(self.workspace_path)

    def ssh_connect(self):
        address = self.line_address.text()
        user = self.line_user.text()
        password = self.line_password.text()
        self.line_connection_status.setText('Connecting...')
        self.text_log.append(f"Connecting to {user}@{address}...")
        QApplication.processEvents()  # Force GUI update

        # Use QTimer to delay the SSH connection attempt
        QTimer.singleShot(100, lambda: self._attempt_ssh_connection(address, user, password))

    def _attempt_ssh_connection(self, address, user, password):
        try:
            self.SSH.connect(address, username=user, password=password)
            self.line_connection_status.setText('Connected')
            self.text_log.append("Connected.")
        except Exception as e:
            self.line_connection_status.setText('Disconnected')
            self.text_log.append(f"Could not connect: {str(e)}")

    def ssh_disconnect(self):
        self.SSH.close()
        self.text_log.append("Disconnecting from Robot.")
        self.line_connection_status.setText('Disconnected')
        return

    def build_and_source(self):
        self.update_workspace_path()
        self.text_log.append(f"Workspace Path: {self.workspace_path}")
        self.execute_and_print(self.SSH, f"cd {self.workspace_path} && catkin build && source devel/setup.bash")

    def record(self):
        self.update_workspace_path()
        self.SSH.exec_command(f"cd {self.workspace_path} && source devel/setup.bash && roslaunch insta360_ros_driver live_process.launch")
        self.execute_and_print(self.SSH, f'cd /home/abanesjo/Desktop/catkin_ws && source devel/setup.bash && echo "Live Topics:\n" && rostopic list')

    def execute_and_print(self, ssh, command):
        ssh = self.SSH
        self.text_log.append("--------------------------------------------")
        try:
            stdin, stdout, stderr = ssh.exec_command(command)

            # Stream stdout
            while True:
                line = stdout.readline()
                if not line:
                    break
                self.text_log.append(line.strip())
                QApplication.processEvents()  # Force GUI update

            # Stream stderr
            while True:
                line = stderr.readline()
                if not line:
                    break
                self.text_log.append(line.strip())
                QApplication.processEvents()  # Force GUI update
    
        except Exception as e:
            QMessageBox.critical(self, 'Error', str(e))
        self.text_log.append("--------------------------------------------")
