# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainwindow.ui'
##
## Created by: Qt User Interface Compiler version 6.7.2
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
        MainWindow.resize(1178, 452)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
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
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Minimum)
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
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.MinimumExpanding, QSizePolicy.Policy.MinimumExpanding)
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
        self.line_password.setEchoMode(QLineEdit.EchoMode.Password)

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
        self.line_bag_path = QLineEdit(self.tab_ros)
        self.line_bag_path.setObjectName(u"line_bag_path")
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.line_bag_path.sizePolicy().hasHeightForWidth())
        self.line_bag_path.setSizePolicy(sizePolicy3)
        self.line_bag_path.setMinimumSize(QSize(0, 0))
        self.line_bag_path.setBaseSize(QSize(1, 0))

        self.gridLayout_2.addWidget(self.line_bag_path, 3, 2, 1, 1)

        self.label_7 = QLabel(self.tab_ros)
        self.label_7.setObjectName(u"label_7")
        sizePolicy4 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.label_7.sizePolicy().hasHeightForWidth())
        self.label_7.setSizePolicy(sizePolicy4)

        self.gridLayout_2.addWidget(self.label_7, 2, 1, 1, 1)

        self.line_bag_nam = QLineEdit(self.tab_ros)
        self.line_bag_nam.setObjectName(u"line_bag_nam")
        sizePolicy3.setHeightForWidth(self.line_bag_nam.sizePolicy().hasHeightForWidth())
        self.line_bag_nam.setSizePolicy(sizePolicy3)

        self.gridLayout_2.addWidget(self.line_bag_nam, 4, 2, 1, 1)

        self.label_8 = QLabel(self.tab_ros)
        self.label_8.setObjectName(u"label_8")

        self.gridLayout_2.addWidget(self.label_8, 4, 1, 1, 1)

        self.label_9 = QLabel(self.tab_ros)
        self.label_9.setObjectName(u"label_9")

        self.gridLayout_2.addWidget(self.label_9, 3, 1, 1, 1)

        self.button_record = QPushButton(self.tab_ros)
        self.button_record.setObjectName(u"button_record")
        sizePolicy5 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.MinimumExpanding)
        sizePolicy5.setHorizontalStretch(0)
        sizePolicy5.setVerticalStretch(0)
        sizePolicy5.setHeightForWidth(self.button_record.sizePolicy().hasHeightForWidth())
        self.button_record.setSizePolicy(sizePolicy5)

        self.gridLayout_2.addWidget(self.button_record, 1, 0, 4, 1)

        self.button_build_source = QPushButton(self.tab_ros)
        self.button_build_source.setObjectName(u"button_build_source")
        sizePolicy3.setHeightForWidth(self.button_build_source.sizePolicy().hasHeightForWidth())
        self.button_build_source.setSizePolicy(sizePolicy3)

        self.gridLayout_2.addWidget(self.button_build_source, 0, 0, 1, 3)

        self.label_5 = QLabel(self.tab_ros)
        self.label_5.setObjectName(u"label_5")
        sizePolicy3.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy3)
        self.label_5.setFrameShape(QFrame.Shape.NoFrame)
        self.label_5.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_2.addWidget(self.label_5, 1, 1, 1, 2)

        self.groupBox = QGroupBox(self.tab_ros)
        self.groupBox.setObjectName(u"groupBox")
        sizePolicy3.setHeightForWidth(self.groupBox.sizePolicy().hasHeightForWidth())
        self.groupBox.setSizePolicy(sizePolicy3)
        self.groupBox.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.verticalLayout_5 = QVBoxLayout(self.groupBox)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.radio_record_true = QRadioButton(self.groupBox)
        self.radio_record_true.setObjectName(u"radio_record_true")
        self.radio_record_true.setChecked(True)

        self.verticalLayout_4.addWidget(self.radio_record_true)

        self.radio_record__false = QRadioButton(self.groupBox)
        self.radio_record__false.setObjectName(u"radio_record__false")
        self.radio_record__false.setAutoFillBackground(False)

        self.verticalLayout_4.addWidget(self.radio_record__false)


        self.verticalLayout_5.addLayout(self.verticalLayout_4)


        self.gridLayout_2.addWidget(self.groupBox, 2, 2, 1, 1)


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
        sizePolicy6 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy6.setHorizontalStretch(1)
        sizePolicy6.setVerticalStretch(0)
        sizePolicy6.setHeightForWidth(self.label_6.sizePolicy().hasHeightForWidth())
        self.label_6.setSizePolicy(sizePolicy6)
        self.label_6.setAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_3.addWidget(self.label_6, 0, 0, 1, 1)

        self.lineEdit_3 = QLineEdit(self.tab_analytics)
        self.lineEdit_3.setObjectName(u"lineEdit_3")
        sizePolicy7 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        sizePolicy7.setHorizontalStretch(1)
        sizePolicy7.setVerticalStretch(0)
        sizePolicy7.setHeightForWidth(self.lineEdit_3.sizePolicy().hasHeightForWidth())
        self.lineEdit_3.setSizePolicy(sizePolicy7)

        self.gridLayout_3.addWidget(self.lineEdit_3, 0, 1, 1, 1)

        self.lineEdit_11 = QLineEdit(self.tab_analytics)
        self.lineEdit_11.setObjectName(u"lineEdit_11")

        self.gridLayout_3.addWidget(self.lineEdit_11, 0, 3, 1, 1)

        self.label_17 = QLabel(self.tab_analytics)
        self.label_17.setObjectName(u"label_17")

        self.gridLayout_3.addWidget(self.label_17, 0, 2, 1, 1)


        self.verticalLayout_6.addLayout(self.gridLayout_3)

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

        self.lineEdit_5 = QLineEdit(self.tab_analytics)
        self.lineEdit_5.setObjectName(u"lineEdit_5")

        self.gridLayout_4.addWidget(self.lineEdit_5, 1, 1, 1, 1)

        self.lineEdit_6 = QLineEdit(self.tab_analytics)
        self.lineEdit_6.setObjectName(u"lineEdit_6")

        self.gridLayout_4.addWidget(self.lineEdit_6, 2, 1, 1, 1)

        self.lineEdit_4 = QLineEdit(self.tab_analytics)
        self.lineEdit_4.setObjectName(u"lineEdit_4")

        self.gridLayout_4.addWidget(self.lineEdit_4, 0, 1, 1, 1)

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
        self.menubar.setGeometry(QRect(0, 0, 1178, 19))
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
        self.line_bag_path.setText(QCoreApplication.translate("MainWindow", u"/mnt/internal/bag/bag_raw", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"Record to Bag File", None))
        self.line_bag_nam.setText(QCoreApplication.translate("MainWindow", u"run1.bag", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Bag File Name", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"Bag File Path", None))
        self.button_record.setText(QCoreApplication.translate("MainWindow", u"Begin Data Collection", None))
        self.button_build_source.setText(QCoreApplication.translate("MainWindow", u"Build Workspace and Source", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Recording Parameters", None))
        self.groupBox.setTitle("")
        self.radio_record_true.setText(QCoreApplication.translate("MainWindow", u"True", None))
        self.radio_record__false.setText(QCoreApplication.translate("MainWindow", u"False", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_ros), QCoreApplication.translate("MainWindow", u"ROS", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Execution Status: ", None))
        self.lineEdit_3.setText(QCoreApplication.translate("MainWindow", u"Not Recording", None))
        self.lineEdit_11.setText(QCoreApplication.translate("MainWindow", u"run1.bag", None))
        self.label_17.setText(QCoreApplication.translate("MainWindow", u"Recording To: ", None))
        self.lineEdit_7.setText(QCoreApplication.translate("MainWindow", u"/livox/lidar", None))
        self.label_16.setText(QCoreApplication.translate("MainWindow", u"Publishing Frequency: ", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"Publishing Frequency: ", None))
        self.label_15.setText(QCoreApplication.translate("MainWindow", u"Publishing Frequency:", None))
        self.label_14.setText(QCoreApplication.translate("MainWindow", u"IMU Topic: ", None))
        self.lineEdit_4.setText(QCoreApplication.translate("MainWindow", u"/front_camera_image/compressed", None))
        self.label_12.setText(QCoreApplication.translate("MainWindow", u"Image Size:", None))
        self.label_13.setText(QCoreApplication.translate("MainWindow", u"PointCloud topic:", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"Image Topic: ", None))
        self.lineEdit_9.setText(QCoreApplication.translate("MainWindow", u"/livox/imu", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_analytics), QCoreApplication.translate("MainWindow", u"Analytics", None))
    # retranslateUi

