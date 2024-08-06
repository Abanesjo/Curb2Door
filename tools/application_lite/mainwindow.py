from PySide6.QtCore import QTimer, Signal, Slot, Qt
from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide6.QtGui import QPixmap, QImage
from ui_mainwindow import Ui_MainWindow

import paramiko
import subprocess
import threading
import os
import re
import numpy as np

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
        self.button_begin_recording.clicked.connect(self.begin_recording)
        self.button_end_recording.clicked.connect(self.end_recording)
        self.button_monitor.clicked.connect(self.monitor)

        self.workspace_path = ""
        self.setup = ""
        self.update_workspace_path()

        self.append_text_signal.connect(self.append_text)

        self.rostopic_processes = {}

    def __del__(self):
        print("Disconnecting from the robot")
        if self.SSH.get_transport() is not None:
            self.SSH.close()

    def closeEvent(self, event):
    # Perform any cleanup before the window closes
        self.stop_sensors()
        self.ssh_disconnect()  # Close SSH connections if open

        # Close any active ROS subscribers
        if hasattr(self, 'image_subscriber'):
            self.image_subscriber.unregister()
        
        # Stop any active rostopic processes
        sessions_to_close = list(self.rostopic_processes.values())
        for session in sessions_to_close:
            session.close()
        self.rostopic_processes.clear()

        super().closeEvent(event)

    def append_text(self, text, text_widget):
        text_widget.append(text)

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
            # Set a timeout for the connection attempt (e.g., 10 seconds)
            self.SSH.connect(address, username=user, password=password, timeout=3)
            self.line_connection_status.setText('Connected')
            self.text_log.append("Connected.")
            os.environ['ROS_MASTER_URI'] = f"http://{self.address}:11311"
            self.text_log.append(f"ROS_MASTER_URI: {os.environ['ROS_MASTER_URI']}")
        except paramiko.ssh_exception.NoValidConnectionsError as e:
            self.line_connection_status.setText('Disconnected')
            self.text_log.append(f"Could not connect: No valid connections: {str(e)}")
        except paramiko.ssh_exception.AuthenticationException as e:
            self.line_connection_status.setText('Disconnected')
            self.text_log.append(f"Could not connect: Authentication failed: {str(e)}")
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
        self.text_log.append("Starting Sensors... (~3s)")

        if self.radio_custom_msg.isChecked():
            lidar_msg = "CustomMsg"
        else:
            lidar_msg = "PointCloud2"

        self.text_log.append(f"Lidar Mode: {lidar_msg}")
        self.SSH.exec_command(f"{self.setup} && roslaunch curb2door bringup.launch lidar_msg:={lidar_msg}")
        QTimer.singleShot(3000, lambda: self.execute_and_print(f'{self.setup} && echo "Live Topics:\n" && rostopic list', self.text_log))

    def stop_sensors(self):
        if self.SSH.get_transport() is not None:
            self.update_workspace_path()
            self.text_log.append("Stoping Sensors...")
            # self.SSH.exec_command(f"{self.setup} && rosnode kill /live_processing /raw_output")
            self.SSH.exec_command("pkill -f ros")

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

        # Start new monitor commands
        image_topic = self.line_image_topic.text()
        self.execute_and_print(f"{self.setup} && rostopic hz {image_topic}", self.text_image_freq, command_id="image")

        lidar_topic = self.line_lidar_topic.text()
        self.execute_and_print(f"{self.setup} && rostopic hz {lidar_topic}", self.text_lidar_freq, command_id="lidar")

        imu_topic = self.line_imu_topic.text()
        self.execute_and_print(f"{self.setup} && rostopic hz {imu_topic}", self.text_imu_freq, command_id="imu")