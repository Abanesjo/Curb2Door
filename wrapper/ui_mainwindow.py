# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainwindow.ui'
##
## Created by: Qt User Interface Compiler version 6.2.4
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QMenuBar, QPushButton, QRadioButton, QSizePolicy,
    QStatusBar, QTabWidget, QTextEdit, QVBoxLayout,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(927, 518)
        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMinimumSize(QSize(800, 400))
        MainWindow.setMaximumSize(QSize(1371, 600))
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        sizePolicy1 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.tabWidget.sizePolicy().hasHeightForWidth())
        self.tabWidget.setSizePolicy(sizePolicy1)
        self.tab_remote = QWidget()
        self.tab_remote.setObjectName(u"tab_remote")
        self.verticalLayout_2 = QVBoxLayout(self.tab_remote)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.label_2 = QLabel(self.tab_remote)
        self.label_2.setObjectName(u"label_2")

        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)

        self.line_user = QLineEdit(self.tab_remote)
        self.line_user.setObjectName(u"line_user")

        self.gridLayout.addWidget(self.line_user, 1, 1, 1, 1)

        self.line_address = QLineEdit(self.tab_remote)
        self.line_address.setObjectName(u"line_address")

        self.gridLayout.addWidget(self.line_address, 0, 1, 1, 1)

        self.button_disconnect = QPushButton(self.tab_remote)
        self.button_disconnect.setObjectName(u"button_disconnect")
        sizePolicy2 = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.button_disconnect.sizePolicy().hasHeightForWidth())
        self.button_disconnect.setSizePolicy(sizePolicy2)

        self.gridLayout.addWidget(self.button_disconnect, 0, 3, 3, 1)

        self.label_3 = QLabel(self.tab_remote)
        self.label_3.setObjectName(u"label_3")

        self.gridLayout.addWidget(self.label_3, 2, 0, 1, 1)

        self.label = QLabel(self.tab_remote)
        self.label.setObjectName(u"label")

        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)

        self.line_password = QLineEdit(self.tab_remote)
        self.line_password.setObjectName(u"line_password")
        self.line_password.setEchoMode(QLineEdit.Password)

        self.gridLayout.addWidget(self.line_password, 2, 1, 1, 1)

        self.button_connect = QPushButton(self.tab_remote)
        self.button_connect.setObjectName(u"button_connect")
        sizePolicy2.setHeightForWidth(self.button_connect.sizePolicy().hasHeightForWidth())
        self.button_connect.setSizePolicy(sizePolicy2)

        self.gridLayout.addWidget(self.button_connect, 0, 2, 3, 1)

        self.line_connection_status = QLineEdit(self.tab_remote)
        self.line_connection_status.setObjectName(u"line_connection_status")

        self.gridLayout.addWidget(self.line_connection_status, 3, 3, 1, 1)

        self.label_4 = QLabel(self.tab_remote)
        self.label_4.setObjectName(u"label_4")

        self.gridLayout.addWidget(self.label_4, 3, 2, 1, 1)


        self.verticalLayout_2.addLayout(self.gridLayout)

        self.tabWidget.addTab(self.tab_remote, "")
        self.tab_ros = QWidget()
        self.tab_ros.setObjectName(u"tab_ros")
        self.verticalLayout_3 = QVBoxLayout(self.tab_ros)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label_18 = QLabel(self.tab_ros)
        self.label_18.setObjectName(u"label_18")

        self.horizontalLayout.addWidget(self.label_18)

        self.line_workspace_path = QLineEdit(self.tab_ros)
        self.line_workspace_path.setObjectName(u"line_workspace_path")

        self.horizontalLayout.addWidget(self.line_workspace_path)


        self.verticalLayout_3.addLayout(self.horizontalLayout)

        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.line_bag_name = QLineEdit(self.tab_ros)
        self.line_bag_name.setObjectName(u"line_bag_name")
        sizePolicy3 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.line_bag_name.sizePolicy().hasHeightForWidth())
        self.line_bag_name.setSizePolicy(sizePolicy3)

        self.gridLayout_2.addWidget(self.line_bag_name, 4, 3, 1, 1)

        self.label_9 = QLabel(self.tab_ros)
        self.label_9.setObjectName(u"label_9")

        self.gridLayout_2.addWidget(self.label_9, 3, 2, 1, 1)

        self.label_8 = QLabel(self.tab_ros)
        self.label_8.setObjectName(u"label_8")

        self.gridLayout_2.addWidget(self.label_8, 4, 2, 1, 1)

        self.button_record = QPushButton(self.tab_ros)
        self.button_record.setObjectName(u"button_record")
        sizePolicy4 = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.MinimumExpanding)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.button_record.sizePolicy().hasHeightForWidth())
        self.button_record.setSizePolicy(sizePolicy4)

        self.gridLayout_2.addWidget(self.button_record, 2, 0, 1, 1)

        self.groupBox = QGroupBox(self.tab_ros)
        self.groupBox.setObjectName(u"groupBox")
        sizePolicy1.setHeightForWidth(self.groupBox.sizePolicy().hasHeightForWidth())
        self.groupBox.setSizePolicy(sizePolicy1)
        self.groupBox.setFlat(False)
        self.groupBox.setCheckable(False)
        self.verticalLayout_5 = QVBoxLayout(self.groupBox)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.radio_record_true = QRadioButton(self.groupBox)
        self.radio_record_true.setObjectName(u"radio_record_true")
        sizePolicy1.setHeightForWidth(self.radio_record_true.sizePolicy().hasHeightForWidth())
        self.radio_record_true.setSizePolicy(sizePolicy1)
        self.radio_record_true.setChecked(True)

        self.verticalLayout_4.addWidget(self.radio_record_true)

        self.radio_record__false = QRadioButton(self.groupBox)
        self.radio_record__false.setObjectName(u"radio_record__false")
        sizePolicy1.setHeightForWidth(self.radio_record__false.sizePolicy().hasHeightForWidth())
        self.radio_record__false.setSizePolicy(sizePolicy1)
        self.radio_record__false.setAutoFillBackground(False)

        self.verticalLayout_4.addWidget(self.radio_record__false)


        self.verticalLayout_5.addLayout(self.verticalLayout_4)


        self.gridLayout_2.addWidget(self.groupBox, 2, 2, 1, 2)

        self.button_build_source = QPushButton(self.tab_ros)
        self.button_build_source.setObjectName(u"button_build_source")
        sizePolicy3.setHeightForWidth(self.button_build_source.sizePolicy().hasHeightForWidth())
        self.button_build_source.setSizePolicy(sizePolicy3)

        self.gridLayout_2.addWidget(self.button_build_source, 0, 0, 1, 4)

        self.line_bag_path = QLineEdit(self.tab_ros)
        self.line_bag_path.setObjectName(u"line_bag_path")
        sizePolicy3.setHeightForWidth(self.line_bag_path.sizePolicy().hasHeightForWidth())
        self.line_bag_path.setSizePolicy(sizePolicy3)
        self.line_bag_path.setMinimumSize(QSize(0, 0))
        self.line_bag_path.setBaseSize(QSize(1, 0))

        self.gridLayout_2.addWidget(self.line_bag_path, 3, 3, 1, 1)

        self.label_5 = QLabel(self.tab_ros)
        self.label_5.setObjectName(u"label_5")
        sizePolicy3.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy3)
        self.label_5.setFrameShape(QFrame.NoFrame)

        self.gridLayout_2.addWidget(self.label_5, 1, 2, 1, 2)

        self.button_chmod = QPushButton(self.tab_ros)
        self.button_chmod.setObjectName(u"button_chmod")

        self.gridLayout_2.addWidget(self.button_chmod, 1, 0, 1, 2)

        self.button_save = QPushButton(self.tab_ros)
        self.button_save.setObjectName(u"button_save")
        sizePolicy4.setHeightForWidth(self.button_save.sizePolicy().hasHeightForWidth())
        self.button_save.setSizePolicy(sizePolicy4)

        self.gridLayout_2.addWidget(self.button_save, 2, 1, 1, 1)

        self.button_preview = QPushButton(self.tab_ros)
        self.button_preview.setObjectName(u"button_preview")
        sizePolicy4.setHeightForWidth(self.button_preview.sizePolicy().hasHeightForWidth())
        self.button_preview.setSizePolicy(sizePolicy4)

        self.gridLayout_2.addWidget(self.button_preview, 3, 0, 2, 1)

        self.button_end_preview = QPushButton(self.tab_ros)
        self.button_end_preview.setObjectName(u"button_end_preview")
        sizePolicy4.setHeightForWidth(self.button_end_preview.sizePolicy().hasHeightForWidth())
        self.button_end_preview.setSizePolicy(sizePolicy4)

        self.gridLayout_2.addWidget(self.button_end_preview, 3, 1, 2, 1)


        self.verticalLayout_3.addLayout(self.gridLayout_2)

        self.tabWidget.addTab(self.tab_ros, "")
        self.tab_analytics = QWidget()
        self.tab_analytics.setObjectName(u"tab_analytics")
        self.verticalLayout_6 = QVBoxLayout(self.tab_analytics)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.gridLayout_3 = QGridLayout()
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.label_6 = QLabel(self.tab_analytics)
        self.label_6.setObjectName(u"label_6")
        sizePolicy5 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy5.setHorizontalStretch(1)
        sizePolicy5.setVerticalStretch(0)
        sizePolicy5.setHeightForWidth(self.label_6.sizePolicy().hasHeightForWidth())
        self.label_6.setSizePolicy(sizePolicy5)

        self.gridLayout_3.addWidget(self.label_6, 0, 0, 1, 1)

        self.line_execution_status = QLineEdit(self.tab_analytics)
        self.line_execution_status.setObjectName(u"line_execution_status")
        sizePolicy6 = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        sizePolicy6.setHorizontalStretch(1)
        sizePolicy6.setVerticalStretch(0)
        sizePolicy6.setHeightForWidth(self.line_execution_status.sizePolicy().hasHeightForWidth())
        self.line_execution_status.setSizePolicy(sizePolicy6)
        self.line_execution_status.setReadOnly(True)

        self.gridLayout_3.addWidget(self.line_execution_status, 0, 1, 1, 1)

        self.line_bag_name_check = QLineEdit(self.tab_analytics)
        self.line_bag_name_check.setObjectName(u"line_bag_name_check")
        self.line_bag_name_check.setReadOnly(True)

        self.gridLayout_3.addWidget(self.line_bag_name_check, 0, 3, 1, 1)

        self.label_17 = QLabel(self.tab_analytics)
        self.label_17.setObjectName(u"label_17")

        self.gridLayout_3.addWidget(self.label_17, 0, 2, 1, 1)


        self.verticalLayout_6.addLayout(self.gridLayout_3)

        self.button_monitor = QPushButton(self.tab_analytics)
        self.button_monitor.setObjectName(u"button_monitor")

        self.verticalLayout_6.addWidget(self.button_monitor)

        self.gridLayout_4 = QGridLayout()
        self.gridLayout_4.setObjectName(u"gridLayout_4")
        self.lineEdit_7 = QLineEdit(self.tab_analytics)
        self.lineEdit_7.setObjectName(u"lineEdit_7")

        self.gridLayout_4.addWidget(self.lineEdit_7, 0, 3, 1, 1)

        self.lineEdit_8 = QLineEdit(self.tab_analytics)
        self.lineEdit_8.setObjectName(u"lineEdit_8")

        self.gridLayout_4.addWidget(self.lineEdit_8, 1, 3, 1, 1)

        self.label_16 = QLabel(self.tab_analytics)
        self.label_16.setObjectName(u"label_16")

        self.gridLayout_4.addWidget(self.label_16, 1, 4, 1, 1)

        self.label_11 = QLabel(self.tab_analytics)
        self.label_11.setObjectName(u"label_11")

        self.gridLayout_4.addWidget(self.label_11, 1, 0, 1, 1)

        self.label_15 = QLabel(self.tab_analytics)
        self.label_15.setObjectName(u"label_15")

        self.gridLayout_4.addWidget(self.label_15, 1, 2, 1, 1)

        self.label_14 = QLabel(self.tab_analytics)
        self.label_14.setObjectName(u"label_14")

        self.gridLayout_4.addWidget(self.label_14, 0, 4, 1, 1)

        self.line_image_freq = QLineEdit(self.tab_analytics)
        self.line_image_freq.setObjectName(u"line_image_freq")

        self.gridLayout_4.addWidget(self.line_image_freq, 1, 1, 1, 1)

        self.line_image_size = QLineEdit(self.tab_analytics)
        self.line_image_size.setObjectName(u"line_image_size")

        self.gridLayout_4.addWidget(self.line_image_size, 2, 1, 1, 1)

        self.line_image_topic = QLineEdit(self.tab_analytics)
        self.line_image_topic.setObjectName(u"line_image_topic")

        self.gridLayout_4.addWidget(self.line_image_topic, 0, 1, 1, 1)

        self.label_12 = QLabel(self.tab_analytics)
        self.label_12.setObjectName(u"label_12")

        self.gridLayout_4.addWidget(self.label_12, 2, 0, 1, 1)

        self.label_13 = QLabel(self.tab_analytics)
        self.label_13.setObjectName(u"label_13")

        self.gridLayout_4.addWidget(self.label_13, 0, 2, 1, 1)

        self.label_10 = QLabel(self.tab_analytics)
        self.label_10.setObjectName(u"label_10")

        self.gridLayout_4.addWidget(self.label_10, 0, 0, 1, 1)

        self.lineEdit_9 = QLineEdit(self.tab_analytics)
        self.lineEdit_9.setObjectName(u"lineEdit_9")

        self.gridLayout_4.addWidget(self.lineEdit_9, 0, 5, 1, 1)

        self.lineEdit_10 = QLineEdit(self.tab_analytics)
        self.lineEdit_10.setObjectName(u"lineEdit_10")

        self.gridLayout_4.addWidget(self.lineEdit_10, 1, 5, 1, 1)


        self.verticalLayout_6.addLayout(self.gridLayout_4)

        self.tabWidget.addTab(self.tab_analytics, "")

        self.verticalLayout.addWidget(self.tabWidget)

        self.text_log = QTextEdit(self.centralwidget)
        self.text_log.setObjectName(u"text_log")

        self.verticalLayout.addWidget(self.text_log)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 927, 20))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Remote Username", None))
        self.line_user.setText(QCoreApplication.translate("MainWindow", u"abanesjo", None))
        self.line_address.setText(QCoreApplication.translate("MainWindow", u"localhost", None))
        self.button_disconnect.setText(QCoreApplication.translate("MainWindow", u"Close Remote Connection", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Remote Password:", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Remote Address", None))
        self.line_password.setText(QCoreApplication.translate("MainWindow", u"Pro0*290@", None))
        self.button_connect.setText(QCoreApplication.translate("MainWindow", u"Connect to Remote", None))
        self.line_connection_status.setText(QCoreApplication.translate("MainWindow", u"Disconnected", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Connection Status:", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_remote), QCoreApplication.translate("MainWindow", u"Remote Connection", None))
        self.label_18.setText(QCoreApplication.translate("MainWindow", u"Catkin Workspace Path:", None))
        self.line_workspace_path.setText(QCoreApplication.translate("MainWindow", u"/home/abanesjo/Desktop/catkin_ws", None))
        self.line_bag_name.setText(QCoreApplication.translate("MainWindow", u"run1.bag", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"Bag File Path", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Bag File Name", None))
        self.button_record.setText(QCoreApplication.translate("MainWindow", u"Begin Data Collection", None))
        self.groupBox.setTitle(QCoreApplication.translate("MainWindow", u"Record to Bag File?", None))
        self.radio_record_true.setText(QCoreApplication.translate("MainWindow", u"True", None))
        self.radio_record__false.setText(QCoreApplication.translate("MainWindow", u"False", None))
        self.button_build_source.setText(QCoreApplication.translate("MainWindow", u"Build Workspace and Source", None))
        self.line_bag_path.setText(QCoreApplication.translate("MainWindow", u"/mnt/internal/bag/bag_raw", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Recording Parameters", None))
        self.button_chmod.setText(QCoreApplication.translate("MainWindow", u"Grant Port Permissions", None))
        self.button_save.setText(QCoreApplication.translate("MainWindow", u"Stop Data Collection", None))
        self.button_preview.setText(QCoreApplication.translate("MainWindow", u"Show Preview", None))
        self.button_end_preview.setText(QCoreApplication.translate("MainWindow", u"End Preview", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_ros), QCoreApplication.translate("MainWindow", u"ROS", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Recording Status: ", None))
        self.line_execution_status.setText(QCoreApplication.translate("MainWindow", u"Recording", None))
        self.line_bag_name_check.setText(QCoreApplication.translate("MainWindow", u"run1.bag", None))
        self.label_17.setText(QCoreApplication.translate("MainWindow", u"Recording To: ", None))
        self.button_monitor.setText(QCoreApplication.translate("MainWindow", u"Monitor", None))
        self.lineEdit_7.setText(QCoreApplication.translate("MainWindow", u"/livox/lidar", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"Publishing Frequency: ", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"Publishing Frequency: ", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"Publishing Frequency:", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"IMU Topic: ", None))
        self.line_image_size.setText("")
        self.line_image_topic.setText(QCoreApplication.translate("MainWindow", u"/front_camera_image/compressed", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"Image Size:", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"PointCloud topic:", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"Image Topic: ", None))
        self.lineEdit_9.setText(QCoreApplication.translate("MainWindow", u"/livox/imu", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_analytics), QCoreApplication.translate("MainWindow", u"Analytics", None))
    # retranslateUi

