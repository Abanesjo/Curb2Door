from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from ui_mainwindow import Ui_MainWindow
from topic_monitor import TopicMonitor

import paramiko
import subprocess
import threading

from sensor_msgs.msg import CompressedImage

class MainWindow(QMainWindow, Ui_MainWindow):
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
        self.button_record.clicked.connect(self.record)
        self.button_save.clicked.connect(self.save)
        self.button_preview.clicked.connect(self.preview)
        self.button_end_preview.clicked.connect(self.end_preview)
        self.button_monitor.clicked.connect(self.monitor)

        self.workspace_path = ""
        self.update_workspace_path()

    def __del__(self):
        print("Disconnecting from the robot")
        self.SSH.close()

    def execute_local(self, command):
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

    def ssh_connect(self):
        self.address = self.line_address.text()
        self.user = self.line_user.text()
        self.password = self.line_password.text()
        self.line_connection_status.setText('Connecting...')
        self.text_log.append(f"Connecting to {self.user}@{self.address}...")
        QApplication.processEvents()

        QTimer.singleShot(100, lambda: self._attempt_ssh_connection(self.address, self.user, self.password))

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
        self.text_log.append("Disconnecting from Remote.")
        self.line_connection_status.setText('Disconnected')
        return

    def build_and_source(self):
        self.update_workspace_path()
        self.text_log.append(f"Workspace Path: {self.workspace_path}")
        self.execute_and_print(f"cd {self.workspace_path} && catkin build && source devel/setup.bash")

    def chmod(self):
        self.text_log.append("-------Granting Port Permissions-----")
        command = """echo SUBSYSTEM=='"usb"', ATTR{idVendor}=='"2e1a"', SYMLINK+='"insta"' | tee /etc/udev/rules.d/99-insta.rules"""
        self.execute_sudo_command(command)
        self.execute_sudo_command("udevadm trigger")
        self.execute_sudo_command("chmod 777 /dev/insta")
        self.text_log.append("--------Port Permission Status---------")
        self.execute_and_print("ls -lR /dev | grep insta")

    def record(self):
        self.update_workspace_path()
        self.SSH.exec_command(f"cd {self.workspace_path} && source devel/setup.bash && roslaunch insta360_ros_driver live_process.launch")
        self.execute_and_print(f'cd {self.workspace_path} && source devel/setup.bash && echo "Live Topics:\n" && rostopic list')

    def save(self):
        self.update_workspace_path()
        self.SSH.exec_command(f"cd {self.workspace_path} && rosnode kill /live_processing /raw_output")

    def preview(self):
        self.text_log.append("Opening Preview (Local)...")
        self.execute_local(f"export ROS_MASTER_URI={self.address}:11311/")
        self.execute_local("rqt_image_view /front_camera_image/compressed &")

    def end_preview(self):
        self.text_log.append("Ending Preview (Local)...")
        self.execute_local(f"rosnode kill rqt_image_view")

    def monitor(self):
        self.text_log.append("Begin monitoring topics")
        image_topic = self.line_image_topic.text()
        self.image_monitor = TopicMonitor(image_topic, CompressedImage)
        self.image_monitor.rate_signal.connect(self.update_image_frequency)
        self.image_monitor.dimension_signal.connect(self.update_image_shape)

    def update_image_frequency(self, rate):
        self.line_image_freq.setText(f"{rate:.3f} Hz")
        
    def update_image_shape(self, width, height):
        self.line_image_size.setText(f"({width}, {height})")

    def execute_and_print(self, command):
        ssh = self.SSH
        try:
            stdin, stdout, stderr = ssh.exec_command(command)

            while True:
                line = stdout.readline()
                if not line:
                    break
                self.text_log.append(line.strip())
                QApplication.processEvents()

            while True:
                line = stderr.readline()
                if not line:
                    break
                self.text_log.append(line.strip())
                QApplication.processEvents()
    
        except Exception as e:
            QMessageBox.critical(self, 'Error', str(e))
        self.text_log.append("------------------------------------------------")