# -*- coding: utf-8 -*-

# Author: Yusuf Yıldız  İTÜ Güneş Arabası Ekibi

from PyQt5 import QtCore, QtGui, QtWidgets
from subprocess import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from shlex import quote
import gae_icons_rc
import os, signal, time, threading, psutil
from termcolor import colored

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PLANNER_TOPIC = "/arduino_node_input"
ROSSERIAL_ERROR = os.path.join(os.getcwd(), "rosserial_error.txt")


class Ui_MainWindow(object):
    allCommands = {
        "roscore": "source /opt/ros/noetic/setup.bash && roscore",
        "rosparam": "source /opt/ros/noetic/setup.bash && rosparam load bridge.yaml && python3 ardino_pub.py",
        "rosserial": "source /opt/ros/noetic/setup.bash && rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=57600",
        "lidar": "source /opt/ros/foxy/setup.bash && ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py",
        "ros2bridge": "source /opt/ros/foxy/setup.bash && ros2 run ros1_bridge parameter_bridge",
        "joy": "source /opt/ros/foxy/setup.bash && ros2 run joy joy_node",
        "mapProvider": "source /home/otonom/AutowareAuto/install/setup.bash && ros2 launch autoware_auto_launch gae_map_provider.launch.py",
        "localizer": "source /home/otonom/AutowareAuto/install/setup.bash && ros2 launch autoware_auto_launch gae_ndt_localizer.launch.py",
        "vehicle": "source /home/otonom/AutowareAuto/install/setup.bash && ros2 launch autoware_auto_launch gae_vehicle.launch.py",
        "visualization": "source /home/otonom/AutowareAuto/install/setup.bash && ros2 launch autoware_auto_launch gae_visualization.launch.py",
        "arduinoController": "source /home/otonom/AutowareAuto/install/setup.bash && python3 gae_planner_controller_arduino.py",
        "joySub": "source /home/otonom/AutowareAuto/install/setup.bash && python3 joy_node_sub_teknofest.py",
        "joyPub": "source /home/otonom/AutowareAuto/install/setup.bash && python3 joy_node_pub_teknofest.py",
        "ros2terminal": 'gnome-terminal -x bash -c "source /opt/ros/foxy/setup.bash; bash"',
        "ros1terminal": 'gnome-terminal -x bash -c "source /opt/ros/noetic/setup.bash; bash"',
        "error": 'gnome-terminal --geometry=60x20 -- bash -c "echo {} && read && exit; bash"',
        "yolo": "python3 commander2.py",
    }

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(500, 300)
        MainWindow.setMinimumSize(QtCore.QSize(450, 250))
        MainWindow.setMaximumSize(QtCore.QSize(700, 450))
        icon = QtGui.QIcon()
        icon.addPixmap(
            QtGui.QPixmap(":/newPrefix/tmp_1d6d8c93-2937-404f-b8ee-0078a5140ff8.png"),
            QtGui.QIcon.Normal,
            QtGui.QIcon.Off,
        )
        MainWindow.setWindowIcon(icon)
        MainWindow.setStyleSheet("#MainWindow{background-color: rgb(85, 87, 83);}")
        MainWindow.setTabShape(QtWidgets.QTabWidget.Triangular)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setStyleSheet(
            "#centralwidget{background-color: rgb(85, 87, 83);}\n" ""
        )
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setMinimumSize(QtCore.QSize(150, 0))
        self.tabWidget.setStyleSheet(
            "\n"
            "#tabWidget{background-color: rgb(85, 87, 83);color: rgb(85, 87, 83);border-color: rgb(85, 87, 83);}\n"
            "\n"
            ""
        )
        self.tabWidget.setTabShape(QtWidgets.QTabWidget.Rounded)
        self.tabWidget.setIconSize(QtCore.QSize(20, 20))
        self.tabWidget.setMovable(False)
        self.tabWidget.setTabBarAutoHide(False)
        self.tabWidget.setObjectName("tabWidget")
        self.ros1_tab = QtWidgets.QWidget()
        self.ros1_tab.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.ros1_tab.setStyleSheet("#ros1_tab{background-color: rgb(85, 87, 83);}")
        self.ros1_tab.setObjectName("ros1_tab")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.ros1_tab)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.checkBox_1_roscore = QtWidgets.QCheckBox(self.ros1_tab)
        self.checkBox_1_roscore.setStyleSheet("color: rgb(233, 185, 110);\n" "")
        self.checkBox_1_roscore.setObjectName("checkBox_1_roscore")
        self.verticalLayout_4.addWidget(self.checkBox_1_roscore)
        spacerItem = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_4.addItem(spacerItem)
        self.checkBox_1_rosparam = QtWidgets.QCheckBox(self.ros1_tab)
        self.checkBox_1_rosparam.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_1_rosparam.setObjectName("checkBox_1_rosparam")
        self.verticalLayout_4.addWidget(self.checkBox_1_rosparam)
        spacerItem1 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_4.addItem(spacerItem1)
        self.checkBox_1_rosserial = QtWidgets.QCheckBox(self.ros1_tab)
        self.checkBox_1_rosserial.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_1_rosserial.setObjectName("checkBox_1_rosserial")
        self.verticalLayout_4.addWidget(self.checkBox_1_rosserial)
        spacerItem2 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_4.addItem(spacerItem2)
        self.checkBox_1_yolo = QtWidgets.QCheckBox(self.ros1_tab)
        self.checkBox_1_yolo.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_1_yolo.setObjectName("checkBox_1_yolo")
        self.verticalLayout_4.addWidget(self.checkBox_1_yolo)
        self.gridLayout_2.addLayout(self.verticalLayout_4, 0, 0, 1, 1)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        spacerItem3 = QtWidgets.QSpacerItem(
            40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum
        )
        self.horizontalLayout_3.addItem(spacerItem3)
        self.pushButton_1_terminate = QtWidgets.QPushButton(self.ros1_tab)
        self.pushButton_1_terminate.setMinimumSize(QtCore.QSize(143, 0))
        self.pushButton_1_terminate.setObjectName("pushButton_1_terminate")
        self.horizontalLayout_3.addWidget(self.pushButton_1_terminate)
        spacerItem4 = QtWidgets.QSpacerItem(
            40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum
        )
        self.horizontalLayout_3.addItem(spacerItem4)
        self.verticalLayout_5.addLayout(self.horizontalLayout_3)
        spacerItem5 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_5.addItem(spacerItem5)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        spacerItem6 = QtWidgets.QSpacerItem(
            40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum
        )
        self.horizontalLayout_2.addItem(spacerItem6)
        self.pushButton_1_open = QtWidgets.QPushButton(self.ros1_tab)
        self.pushButton_1_open.setObjectName("pushButton_1_open")
        self.horizontalLayout_2.addWidget(self.pushButton_1_open)
        spacerItem7 = QtWidgets.QSpacerItem(
            40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum
        )
        self.horizontalLayout_2.addItem(spacerItem7)
        self.verticalLayout_5.addLayout(self.horizontalLayout_2)
        spacerItem8 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_5.addItem(spacerItem8)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem9 = QtWidgets.QSpacerItem(
            40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum
        )
        self.horizontalLayout.addItem(spacerItem9)
        self.label = QtWidgets.QLabel(self.ros1_tab)
        self.label.setMinimumSize(QtCore.QSize(170, 100))
        self.label.setStyleSheet(
            "border-image: url(:/newPrefix/tmp_1d6d8c93-2937-404f-b8ee-0078a5140ff8.png);"
        )
        self.label.setText("")
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        spacerItem10 = QtWidgets.QSpacerItem(
            40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum
        )
        self.horizontalLayout.addItem(spacerItem10)
        self.verticalLayout_5.addLayout(self.horizontalLayout)
        self.gridLayout_2.addLayout(self.verticalLayout_5, 0, 1, 1, 1)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(
            QtGui.QPixmap(":/newPrefix/ros1_imge.png"),
            QtGui.QIcon.Normal,
            QtGui.QIcon.Off,
        )
        self.tabWidget.addTab(self.ros1_tab, icon1, "")
        self.ros2_tab = QtWidgets.QWidget()
        self.ros2_tab.setStyleSheet(" #ros2_tab{background-color: rgb(85, 87, 83);}")
        self.ros2_tab.setObjectName("ros2_tab")
        self.gridLayout = QtWidgets.QGridLayout(self.ros2_tab)
        self.gridLayout.setObjectName("gridLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.checkBox_2_lidar = QtWidgets.QCheckBox(self.ros2_tab)
        self.checkBox_2_lidar.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.checkBox_2_lidar.setAutoFillBackground(False)
        self.checkBox_2_lidar.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_2_lidar.setObjectName("checkBox_2_lidar")
        self.verticalLayout.addWidget(self.checkBox_2_lidar)
        spacerItem11 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout.addItem(spacerItem11)
        self.checkBox_2_bridge = QtWidgets.QCheckBox(self.ros2_tab)
        self.checkBox_2_bridge.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_2_bridge.setObjectName("checkBox_2_bridge")
        self.verticalLayout.addWidget(self.checkBox_2_bridge)
        spacerItem12 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout.addItem(spacerItem12)
        self.checkBox_2_joy = QtWidgets.QCheckBox(self.ros2_tab)
        self.checkBox_2_joy.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_2_joy.setObjectName("checkBox_2_joy")
        self.verticalLayout.addWidget(self.checkBox_2_joy)
        spacerItem13 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout.addItem(spacerItem13)
        self.checkBox_2_joypub = QtWidgets.QCheckBox(self.ros2_tab)
        self.checkBox_2_joypub.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_2_joypub.setObjectName("checkBox_2_joypub")
        self.verticalLayout.addWidget(self.checkBox_2_joypub)
        spacerItem14 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout.addItem(spacerItem14)
        self.checkBox_2_joysub = QtWidgets.QCheckBox(self.ros2_tab)
        self.checkBox_2_joysub.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_2_joysub.setObjectName("checkBox_2_joysub")
        self.verticalLayout.addWidget(self.checkBox_2_joysub)
        self.gridLayout.addLayout(self.verticalLayout, 0, 0, 1, 1)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.checkBox_2_map = QtWidgets.QCheckBox(self.ros2_tab)
        self.checkBox_2_map.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_2_map.setObjectName("checkBox_2_map")
        self.verticalLayout_2.addWidget(self.checkBox_2_map)
        spacerItem15 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_2.addItem(spacerItem15)
        self.checkBox_2_localizer = QtWidgets.QCheckBox(self.ros2_tab)
        self.checkBox_2_localizer.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_2_localizer.setObjectName("checkBox_2_localizer")
        self.verticalLayout_2.addWidget(self.checkBox_2_localizer)
        spacerItem16 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_2.addItem(spacerItem16)
        self.checkBox_2_vehicle = QtWidgets.QCheckBox(self.ros2_tab)
        self.checkBox_2_vehicle.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_2_vehicle.setObjectName("checkBox_2_vehicle")
        self.verticalLayout_2.addWidget(self.checkBox_2_vehicle)
        spacerItem17 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_2.addItem(spacerItem17)
        self.checkBox_2_visualization = QtWidgets.QCheckBox(self.ros2_tab)
        self.checkBox_2_visualization.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_2_visualization.setObjectName("checkBox_2_visualization")
        self.verticalLayout_2.addWidget(self.checkBox_2_visualization)
        spacerItem18 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_2.addItem(spacerItem18)
        self.checkBox_2_ardctrl = QtWidgets.QCheckBox(self.ros2_tab)
        self.checkBox_2_ardctrl.setStyleSheet("color: rgb(233, 185, 110);")
        self.checkBox_2_ardctrl.setObjectName("checkBox_2_ardctrl")
        self.verticalLayout_2.addWidget(self.checkBox_2_ardctrl)
        self.gridLayout.addLayout(self.verticalLayout_2, 0, 1, 1, 1)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        spacerItem19 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_3.addItem(spacerItem19)
        self.pushButton_2_terminate = QtWidgets.QPushButton(self.ros2_tab)
        self.pushButton_2_terminate.setObjectName("pushButton_2_terminate")
        self.verticalLayout_3.addWidget(self.pushButton_2_terminate)
        spacerItem20 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_3.addItem(spacerItem20)
        self.pushButton_2_open = QtWidgets.QPushButton(self.ros2_tab)
        self.pushButton_2_open.setMinimumSize(QtCore.QSize(0, 0))
        self.pushButton_2_open.setObjectName("pushButton_2_open")
        self.verticalLayout_3.addWidget(self.pushButton_2_open)
        spacerItem21 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_3.addItem(spacerItem21)
        self.label_2 = QtWidgets.QLabel(self.ros2_tab)
        self.label_2.setMinimumSize(QtCore.QSize(0, 100))
        self.label_2.setStyleSheet(
            "border-image: url(:/newPrefix/tmp_1d6d8c93-2937-404f-b8ee-0078a5140ff8.png);"
        )
        self.label_2.setText("")
        self.label_2.setObjectName("label_2")
        self.verticalLayout_3.addWidget(self.label_2)
        self.gridLayout.addLayout(self.verticalLayout_3, 0, 2, 1, 1)
        icon2 = QtGui.QIcon()
        icon2.addPixmap(
            QtGui.QPixmap(":/newPrefix/ros2_imag.png"),
            QtGui.QIcon.Normal,
            QtGui.QIcon.Off,
        )
        self.tabWidget.addTab(self.ros2_tab, icon2, "")
        self.utils_tab = QtWidgets.QWidget()
        self.utils_tab.setStyleSheet("#utils_tab{background-color: rgb(85, 87, 83);}")
        self.utils_tab.setObjectName("utils_tab")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.utils_tab)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout()
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.label_speed = QtWidgets.QLabel(self.utils_tab)
        self.label_speed.setStyleSheet("color: rgb(233, 185, 110);")
        self.label_speed.setObjectName("label_speed")
        self.verticalLayout_7.addWidget(self.label_speed)
        self.lcdNumber = QtWidgets.QLCDNumber(self.utils_tab)
        self.lcdNumber.setBaseSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setItalic(False)
        font.setUnderline(False)
        font.setWeight(75)
        font.setStrikeOut(False)
        font.setKerning(False)
        font.setStyleStrategy(QtGui.QFont.PreferDefault)
        self.lcdNumber.setFont(font)
        self.lcdNumber.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.lcdNumber.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.lcdNumber.setFrameShadow(QtWidgets.QFrame.Plain)
        self.lcdNumber.setLineWidth(1)
        self.lcdNumber.setSmallDecimalPoint(False)
        self.lcdNumber.setDigitCount(5)
        self.lcdNumber.setObjectName("lcdNumber")
        self.verticalLayout_7.addWidget(self.lcdNumber)
        self.label_angle = QtWidgets.QLabel(self.utils_tab)
        self.label_angle.setStyleSheet("color: rgb(233, 185, 110);")
        self.label_angle.setObjectName("label_angle")
        self.verticalLayout_7.addWidget(self.label_angle)
        self.lcdNumber_2 = QtWidgets.QLCDNumber(self.utils_tab)
        self.lcdNumber_2.setStyleSheet("")
        self.lcdNumber_2.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.lcdNumber_2.setObjectName("lcdNumber_2")
        self.verticalLayout_7.addWidget(self.lcdNumber_2)
        self.label_regen = QtWidgets.QLabel(self.utils_tab)
        self.label_regen.setStyleSheet("color: rgb(233, 185, 110);")
        self.label_regen.setObjectName("label_regen")
        self.verticalLayout_7.addWidget(self.label_regen)
        self.lcdNumber_3 = QtWidgets.QLCDNumber(self.utils_tab)
        self.lcdNumber_3.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.lcdNumber_3.setObjectName("lcdNumber_3")
        self.verticalLayout_7.addWidget(self.lcdNumber_3)
        self.horizontalLayout_5.addLayout(self.verticalLayout_7)
        spacerItem22 = QtWidgets.QSpacerItem(
            40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum
        )
        self.horizontalLayout_5.addItem(spacerItem22)
        self.verticalLayout_9 = QtWidgets.QVBoxLayout()
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.label_gear = QtWidgets.QLabel(self.utils_tab)
        self.label_gear.setStyleSheet("color: rgb(233, 185, 110);")
        self.label_gear.setObjectName("label_gear")
        self.verticalLayout_9.addWidget(self.label_gear)
        self.label_3_gear = QtWidgets.QLabel(self.utils_tab)
        font = QtGui.QFont()
        font.setPointSize(13)
        self.label_3_gear.setFont(font)
        self.label_3_gear.setStyleSheet("color: rgb(255, 255, 255);")
        self.label_3_gear.setText("")
        self.label_3_gear.setObjectName("label_3_gear")
        self.verticalLayout_9.addWidget(self.label_3_gear)
        spacerItem23 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_9.addItem(spacerItem23)
        self.label_mode = QtWidgets.QLabel(self.utils_tab)
        self.label_mode.setStyleSheet("color: rgb(233, 185, 110);")
        self.label_mode.setObjectName("label_mode")
        self.verticalLayout_9.addWidget(self.label_mode)
        self.label_3_mode = QtWidgets.QLabel(self.utils_tab)
        font = QtGui.QFont()
        font.setPointSize(13)
        self.label_3_mode.setFont(font)
        self.label_3_mode.setStyleSheet("color: rgb(255, 255, 255);")
        self.label_3_mode.setText("")
        self.label_3_mode.setObjectName("label_3_mode")
        self.verticalLayout_9.addWidget(self.label_3_mode)
        spacerItem24 = QtWidgets.QSpacerItem(
            20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding
        )
        self.verticalLayout_9.addItem(spacerItem24)
        self.label_engine = QtWidgets.QLabel(self.utils_tab)
        self.label_engine.setStyleSheet("color: rgb(233, 185, 110);")
        self.label_engine.setObjectName("label_engine")
        self.verticalLayout_9.addWidget(self.label_engine)
        self.label_3_mode_2 = QtWidgets.QLabel(self.utils_tab)
        font = QtGui.QFont()
        font.setPointSize(13)
        self.label_3_mode_2.setFont(font)
        self.label_3_mode_2.setStyleSheet("color: rgb(255, 255, 255);")
        self.label_3_mode_2.setText("")
        self.label_3_mode_2.setObjectName("label_3_mode_2")
        self.verticalLayout_9.addWidget(self.label_3_mode_2)
        self.horizontalLayout_5.addLayout(self.verticalLayout_9)
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.horizontalLayout_5.addLayout(self.verticalLayout_6)
        spacerItem25 = QtWidgets.QSpacerItem(
            40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum
        )
        self.horizontalLayout_5.addItem(spacerItem25)
        self.gridLayout_4.addLayout(self.horizontalLayout_5, 0, 0, 1, 1)
        self.tabWidget.addTab(self.utils_tab, icon, "")
        self.gridLayout_3.addWidget(self.tabWidget, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(2)

        self.checkBox_1_roscore.toggled["bool"].connect(self.roscore)
        self.checkBox_1_rosparam.toggled["bool"].connect(self.rosparam)
        self.checkBox_1_rosserial.toggled["bool"].connect(self.rosserial)
        self.checkBox_1_yolo.toggled["bool"].connect(self.yolo)
        self.checkBox_2_lidar.toggled["bool"].connect(self.lidar)
        self.checkBox_2_bridge.toggled["bool"].connect(self.ros2_bridge)
        self.checkBox_2_joy.toggled["bool"].connect(self.joy)
        self.checkBox_2_joypub.toggled["bool"].connect(self.joy_pub)
        self.checkBox_2_joysub.toggled["bool"].connect(self.joy_sub)
        self.checkBox_2_map.toggled["bool"].connect(self.map_provider)
        self.checkBox_2_localizer.toggled["bool"].connect(self.ndt_localizer)
        self.checkBox_2_vehicle.toggled["bool"].connect(self.vehicle)
        self.checkBox_2_visualization.toggled["bool"].connect(self.visualization)
        self.checkBox_2_ardctrl.toggled["bool"].connect(self.ard_ctrl)

        self.pushButton_1_terminate.clicked.connect(self.ros1_terminator)
        self.pushButton_1_open.clicked.connect(self.ros1_opener)
        self.pushButton_2_terminate.clicked.connect(self.ros2_terminator)
        self.pushButton_2_open.clicked.connect(self.ros2_opener)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.checkBox_1_roscore.setToolTip(self.allCommands["roscore"])
        self.checkBox_1_rosparam.setToolTip(self.allCommands["rosparam"])
        self.checkBox_1_rosserial.setToolTip(self.allCommands["rosserial"])
        self.checkBox_1_yolo.setToolTip(self.allCommands["yolo"])
        self.checkBox_2_lidar.setToolTip(self.allCommands["lidar"])
        self.checkBox_2_bridge.setToolTip(self.allCommands["ros2bridge"])
        self.checkBox_2_joy.setToolTip(self.allCommands["joy"])
        self.checkBox_2_joypub.setToolTip(self.allCommands["joyPub"])
        self.checkBox_2_joysub.setToolTip(self.allCommands["joySub"])
        self.checkBox_2_map.setToolTip(self.allCommands["mapProvider"])
        self.checkBox_2_localizer.setToolTip(self.allCommands["localizer"])
        self.checkBox_2_vehicle.setToolTip(self.allCommands["vehicle"])
        self.checkBox_2_visualization.setToolTip(self.allCommands["visualization"])
        self.checkBox_2_ardctrl.setToolTip(self.allCommands["arduinoController"])

        self.lidarObject = None
        self.rosbridgeObject = None
        self.joyObject = None
        self.joypubObject = None
        self.joysubObject = None
        self.mapProviderObject = None
        self.localizerObject = None
        self.vehicleObject = None
        self.visualizationObject = None
        self.ardctrlObject = None
        self.ros2openerObject = None

        self.roscoreObject = None
        self.rosparamObject = None
        self.rosserialObject = None
        self.yoloObject = None
        self.ros1openerObject = None

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Autonomous II"))
        self.checkBox_1_roscore.setText(_translate("MainWindow", "Roscore"))
        self.checkBox_1_rosparam.setText(
            _translate("MainWindow", "Rosparam and Arduino Pub")
        )
        self.checkBox_1_rosserial.setText(_translate("MainWindow", "Rosserial Arduino"))
        self.checkBox_1_yolo.setText(_translate("MainWindow", "YOLO"))
        self.pushButton_1_terminate.setText(_translate("MainWindow", "Terminate All"))
        self.pushButton_1_open.setText(_translate("MainWindow", "Open New Terminal"))
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.ros1_tab), _translate("MainWindow", "ROS 1")
        )
        self.checkBox_2_lidar.setText(_translate("MainWindow", "Velodyne LIDAR"))
        self.checkBox_2_bridge.setText(_translate("MainWindow", "ROS1 Bridge"))
        self.checkBox_2_joy.setText(_translate("MainWindow", "Joy Node"))
        self.checkBox_2_joypub.setText(_translate("MainWindow", "Joy Publisher"))
        self.checkBox_2_joysub.setText(_translate("MainWindow", "Joy Subscriber"))
        self.checkBox_2_map.setText(_translate("MainWindow", "Map Provider"))
        self.checkBox_2_localizer.setText(_translate("MainWindow", "NDT Localizer"))
        self.checkBox_2_vehicle.setText(_translate("MainWindow", "Vehicle"))
        self.checkBox_2_visualization.setText(_translate("MainWindow", "Visualization"))
        self.checkBox_2_ardctrl.setText(_translate("MainWindow", "Arduino Controller"))
        self.pushButton_2_terminate.setText(_translate("MainWindow", "Terminate All"))
        self.pushButton_2_open.setText(_translate("MainWindow", "Open Nem Terminal"))
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.ros2_tab), _translate("MainWindow", "ROS 2")
        )
        self.label_speed.setText(_translate("MainWindow", "Speed:"))
        self.label_angle.setText(_translate("MainWindow", "Steering Angle:"))
        self.label_regen.setText(_translate("MainWindow", "Regen:"))
        self.label_gear.setText(_translate("MainWindow", "Gear:"))
        self.label_mode.setText(_translate("MainWindow", "Autonomous Mode:"))
        self.label_engine.setText(_translate("MainWindow", "Engine Mode:"))
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.utils_tab), _translate("MainWindow", "UI UTILS")
        )

    def lidar(self, state):

        if state:
            self.lidarObject = Popen(
                [self.allCommands["lidar"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom",
            )
            try:
                out, err = self.lidarObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.lidarObject,
                        self.checkBox_2_lidar,
                        "LIDAR",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("LIDAR", self.checkBox_2_lidar, err)
        else:
            if self.lidarObject.poll() == None:
                os.killpg(os.getpgid(self.lidarObject.pid), signal.SIGTERM)

    def ros2_bridge(self, state):

        if state:
            self.rosbridgeObject = Popen(
                [self.allCommands["ros2bridge"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom",
            )
            try:
                out, err = self.rosbridgeObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.rosbridgeObject,
                        self.checkBox_2_bridge,
                        "ROS BRIDGE",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("ROS BRIDGE", self.checkBox_2_bridge, err)
        else:
            if self.rosbridgeObject.poll() == None:
                os.killpg(os.getpgid(self.rosbridgeObject.pid), signal.SIGTERM)

    def joy(self, state):

        if state:
            self.joyObject = Popen(
                [self.allCommands["joy"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom",
            )
            try:
                out, err = self.joyObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.joyObject,
                        self.checkBox_2_joy,
                        "JOY",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("JOY", self.checkBox_2_joy, err)
        else:
            if self.joyObject.poll() == None:
                os.killpg(os.getpgid(self.joyObject.pid), signal.SIGTERM)

    def joy_pub(self, state):
        if state:
            self.joypubObject = Popen(
                [self.allCommands["joyPub"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom/AutowareAuto/install/autoware_auto_launch/share/autoware_auto_launch/launch",
            )
            try:
                out, err = self.joypubObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.joypubObject,
                        self.checkBox_2_joypub,
                        "JOY PUBLISHER",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("JOY PUBLISHER", self.checkBox_2_joypub, err)
        else:
            if self.joypubObject.poll() == None:
                os.killpg(os.getpgid(self.joypubObject.pid), signal.SIGTERM)

    def joy_sub(self, state):
        if state:
            self.joysubObject = Popen(
                [self.allCommands["joySub"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom/AutowareAuto/install/autoware_auto_launch/share/autoware_auto_launch/launch",
            )
            try:
                out, err = self.joysubObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.joysubObject,
                        self.checkBox_2_joysub,
                        "JOY SUBSCRIBER",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("JOY SUBSCRIBER", self.checkBox_2_joysub, err)
        else:
            if self.joysubObject.poll() == None:
                os.killpg(os.getpgid(self.joysubObject.pid), signal.SIGTERM)

    def map_provider(self, state):

        if state:
            self.mapProviderObject = Popen(
                [self.allCommands["mapProvider"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom",
            )
            try:
                out, err = self.mapProviderObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.mapProviderObject,
                        self.checkBox_2_map,
                        "MAP PROVIDER",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("MAP PROVIDER", self.checkBox_2_map, err)
        else:
            if self.mapProviderObject.poll() == None:
                os.killpg(os.getpgid(self.mapProviderObject.pid), signal.SIGTERM)

    def ndt_localizer(self, state):

        if state:
            self.localizerObject = Popen(
                [self.allCommands["localizer"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom",
            )
            try:
                out, err = self.localizerObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.localizerObject,
                        self.checkBox_2_localizer,
                        "NDT LOCALIZATION",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("NDT LOCALIZER", self.checkBox_2_localizer, err)
        else:
            if self.localizerObject.poll() == None:
                os.killpg(os.getpgid(self.localizerObject.pid), signal.SIGTERM)

    def vehicle(self, state):

        if state:
            self.vehicleObject = Popen(
                [self.allCommands["vehicle"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom",
            )
            try:
                out, err = self.vehicleObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.vehicleObject,
                        self.checkBox_2_vehicle,
                        "VEHICLE",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("VEHICLE", self.checkBox_2_vehicle, err)
        else:
            if self.vehicleObject.poll() == None:
                os.killpg(os.getpgid(self.vehicleObject.pid), signal.SIGTERM)

    def visualization(self, state):

        if state:
            self.visualizationObject = Popen(
                [self.allCommands["visualization"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom",
            )
            try:
                out, err = self.visualizationObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.visualizationObject,
                        self.checkBox_2_visualization,
                        "VISUALIZATION",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("VISUALIZATION", self.checkBox_2_visualization, err)
        else:
            if self.visualizationObject.poll() == None:
                os.killpg(os.getpgid(self.visualizationObject.pid), signal.SIGTERM)

    def ard_ctrl(self, state):

        if state:
            self.ardctrlObject = Popen(
                [self.allCommands["arduinoController"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom/AutowareAuto/install/autoware_auto_launch/share/autoware_auto_launch/launch",
            )
            try:
                out, err = self.ardctrlObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.ardctrlObject,
                        self.checkBox_2_ardctrl,
                        "ARDUINO CONTROLLER",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("ARDUINO CONTROLLER", self.checkBox_2_ardctrl, err)
        else:
            if self.ardctrlObject.poll() == None:
                os.killpg(os.getpgid(self.ardctrlObject.pid), signal.SIGTERM)

    def ros2_terminator(self):
        self.checkBox_2_ardctrl.setChecked(False)
        self.checkBox_2_bridge.setChecked(False)
        self.checkBox_2_joy.setChecked(False)
        self.checkBox_2_vehicle.setChecked(False)
        self.checkBox_2_joypub.setChecked(False)
        self.checkBox_2_joysub.setChecked(False)
        self.checkBox_2_lidar.setChecked(False)
        self.checkBox_2_localizer.setChecked(False)
        self.checkBox_2_map.setChecked(False)
        self.checkBox_2_visualization.setChecked(False)

        self.statusbar.showMessage("All ROS2 Nodes Terminated.", 1500)
        self.statusbar.setStyleSheet("color:rgb(255, 255, 255);")

    def ros2_opener(self):
        self.ros2openerObject = Popen(
            [self.allCommands["ros2terminal"]],
            stdout=PIPE,
            stderr=PIPE,
            shell=True,
            text=True,
            executable="/bin/bash",
            cwd="/home/otonom",
        )

        self.statusbar.showMessage("New ROS2 Terminal Created.", 1500)
        self.statusbar.setStyleSheet("color:rgb(255, 255, 255);")

    def roscore(self, state):
        if state:
            self.roscoreObject = Popen(
                [self.allCommands["roscore"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom",
            )
            try:
                out, err = self.roscoreObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.roscoreObject,
                        self.checkBox_1_roscore,
                        "ROSCORE",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("ROSCORE", self.checkBox_1_roscore, err)
        else:
            if self.roscoreObject.poll() == None:
                os.killpg(os.getpgid(self.roscoreObject.pid), signal.SIGTERM)

    def rosparam(self, state):

        if state:
            self.rosparamObject = Popen(
                [self.allCommands["rosparam"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom/bridge",
            )
            try:
                out, err = self.rosparamObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.rosparamObject,
                        self.checkBox_1_rosparam,
                        "ROSPARAM",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("ROSPARAM", self.checkBox_1_rosparam, err)
        else:
            if self.rosparamObject.poll() == None:
                os.killpg(os.getpgid(self.rosparamObject.pid), signal.SIGTERM)

    def rosserial(self, state):

        if state:
            with open(ROSSERIAL_ERROR, "r+") as f:
                self.rosserialObject = Popen(
                    [self.allCommands["rosserial"]],
                    stdout=PIPE,
                    stderr=f,
                    shell=True,
                    text=True,
                    executable="/bin/bash",
                    preexec_fn=os.setsid,
                    cwd="/home/otonom",
                )
                time.sleep(1)
                os.killpg(os.getpgid(self.rosserialObject.pid), signal.SIGTERM)
                time.sleep(4)
                err = f.read()
                if err:
                    self.raise_error("ROSSERIAL", self.checkBox_1_rosserial, err)
                else:
                    self.rosserialObject = Popen(
                        [self.allCommands["rosserial"]],
                        stdout=PIPE,
                        stderr=PIPE,
                        shell=True,
                        text=True,
                        executable="/bin/bash",
                        preexec_fn=os.setsid,
                        cwd="/home/otonom",
                    )
                    process_thread = threading.Thread(
                        target=self.pid_controller,
                        args=(
                            self.rosserialObject,
                            self.checkBox_1_rosserial,
                            "ROSSERIAL",
                        ),
                        daemon=True,
                    )
                    process_thread.start()
        else:
            if self.rosserialObject.poll() == None:
                os.killpg(os.getpgid(self.rosserialObject.pid), signal.SIGTERM)

    def yolo(self, state):
        if state:
            self.yoloObject = Popen(
                [self.allCommands["yolo"]],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
                preexec_fn=os.setsid,
                cwd="/home/otonom/perception/rosweb/scripts",
            )
            try:
                out, err = self.yoloObject.communicate(timeout=2)
            except TimeoutExpired:
                process_thread = threading.Thread(
                    target=self.pid_controller,
                    args=(
                        self.yoloObject,
                        self.checkBox_1_yolo,
                        "YOLO",
                    ),
                    daemon=True,
                )
                process_thread.start()
            else:
                self.raise_error("YOLO", self.checkBox_1_yolo, err)
        else:
            if self.yoloObject.poll() == None:
                os.killpg(os.getpgid(self.yoloObject.pid), signal.SIGTERM)

    def ros1_terminator(self):
        self.checkBox_1_roscore.setChecked(False)
        self.checkBox_1_rosparam.setChecked(False)
        self.checkBox_1_rosserial.setChecked(False)
        self.checkBox_1_yolo.setChecked(False)

        self.statusbar.showMessage("All ROS1 Nodes Terminated.", 1500)
        self.statusbar.setStyleSheet("color:rgb(255, 255, 255);")

    def ros1_opener(self):
        self.ros1openerObject = Popen(
            [self.allCommands["ros1terminal"]],
            stdout=PIPE,
            stderr=PIPE,
            shell=True,
            text=True,
            executable="/bin/bash",
            cwd="/home/otonom",
        )

        self.statusbar.showMessage("New ROS1 Terminal Created.", 1500)
        self.statusbar.setStyleSheet("color:rgb(255, 255, 255);")

    def pid_control(self, obj, checkBox, name):
        controller = ErrorThread(obj, checkBox, name)
        controller.error_signal.connect(self.raise_error)
        controller.start()

    def raise_error(self, name, checkBox, err):
        if err:
            Popen(
                [
                    self.allCommands["error"].format(
                        quote(
                            colored(f"{name} ERROR\n", "red")
                            + err.replace('"', "^").replace("'", "*")
                        )
                    )
                ],
                stdout=PIPE,
                stderr=PIPE,
                shell=True,
                text=True,
                executable="/bin/bash",
            )
            self.statusbar.showMessage(f"{name} command couldn't be run.", 1500)
            self.statusbar.setStyleSheet("color:rgb(255, 255, 255);")
            checkBox.setChecked(False)


class Ui_Util(Node):
    def __init__(self, ui):
        super().__init__("gae_node")
        self.sub = self.create_subscription(
            String, PLANNER_TOPIC, self.listener_callback, 10
        )
        self.my_list = []
        self.ui = ui

    def listener_callback(self, msg):
        self.my_list = msg.data.split(";")
        self.ui.lcdNumber.display(int(self.my_list[0]))
        self.ui.lcdNumber_2.display((-int(self.my_list[1]) + 1800) / 20)
        self.ui.lcdNumber_3.display(int(self.my_list[2]) / 10)

        if int(self.my_list[3]) == 0:
            self.ui.label_3_gear.setText("NEUTRAL")
        elif int(self.my_list[3]) == 1:
            self.ui.label_3_gear.setText("FORWARD")
        elif int(self.my_list[3]) == 2:
            self.ui.label_3_gear.setText("REVERSE")

        if int(self.my_list[4]) == 0:
            self.ui.label_3_mode.setText("REMOTE")
        elif int(self.my_list[4]) == 1:
            self.ui.label_3_mode.setText("AUTONOMOUS")

        if int(self.my_list[5]) == 0:
            self.ui.label_3_mode_2.setText("CURRENT")
        elif int(self.my_list[5]) == 1:
            self.ui.label_3_mode_2.setText("RPM")


class ErrorThread(QObject):
    error_signal = pyqtSignal(str, QtWidgets.QCheckBox, str)

    def __init__(self, name, checkBox, obj):
        super().__init__()
        self.name = name
        self.checkBox = checkBox
        self.object = obj
        self.run()

    def run(self):
        process_state = True

        while process_state:
            time.sleep(0.5)
            process_state = psutil.pid_exists(self.object.pid)

        self.error_signal.emit(self.name, self.checkBox, self.object.stderr.read())


def ROS(obj):
    rclpy.spin(obj)
    obj.destroy_node()
    rclpy.shutdown()


def UI(args=None):
    import sys

    app = QApplication(sys.argv)
    mainWindow = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(mainWindow)

    rclpy.init(args=args)
    ros = Ui_Util(ui)

    ros = threading.Thread(target=ROS, args=(ros,), daemon=True)
    ros.start()

    mainWindow.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    UI()
