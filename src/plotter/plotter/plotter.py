import threading
from collections import deque
import numpy as np
import pyqtgraph as pg
import pyqtgraph.exporters as pg_exporters
from PyQt5 import QtCore
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import QPushButton, QVBoxLayout, QWidget, QTabWidget,QHBoxLayout, QLineEdit, QLabel, QShortcut
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from vesc_msgs.msg import VescStateStamped
from scipy.spatial.transform import Rotation
import csv
import os
import scipy.io as sio
from datetime import datetime
import subprocess
import signal

class PlotterNode(Node):
    def __init__(self):
        super().__init__('PlotterNode')

        self.lock = threading.Lock()
        self.data = {}
        self.start_time = None
        self.plotting_active = False
        self.plotting_paused = False

        # Smaller buffers for performance
        self.add_buffer('x_odom', maxlen=2000)
        self.add_buffer('y_odom', maxlen=2000)
        self.add_buffer('yaw_odom', maxlen=3000)
        self.add_buffer('vel', maxlen=3000)
        self.add_buffer('x_accel', maxlen=1500)
        self.add_buffer('y_accel', maxlen=1500)
        self.add_buffer('erpm', maxlen=1500)
        self.add_buffer('input_erpm', maxlen=3000)
        self.add_buffer('input_vel', maxlen=3000)
        self.add_buffer('vel_error', maxlen=3000)
        self.add_buffer('exp_x', maxlen=2000)
        self.add_buffer('exp_y', maxlen=2000)
        self.add_buffer('x_error', maxlen=2000)
        self.add_buffer('y_error', maxlen=2000)

        self.traj_x = deque(maxlen=10000)
        self.traj_y = deque(maxlen=10000)
        self.exp_x = deque(maxlen=10000)
        self.exp_y = deque(maxlen=10000)

        # Track start time
        self.start_time = None

        # ROS bag process
        self.rosbag_process = None

        # Create main widget with button
        self.main_widget = QWidget()
        self.main_widget.setWindowTitle('F1TENTH Plotter')
        self.main_widget.resize(1400, 900)
        self.main_layout = QVBoxLayout()
        self.main_widget.setLayout(self.main_layout)

        self.control_layout = QHBoxLayout()

        self.toggle_button = QPushButton("Start")
        self.toggle_button.setMaximumHeight(50)
        self.toggle_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        self.toggle_button.clicked.connect(self.toggle_plotting)
        self.main_layout.addWidget(self.toggle_button)

        self.filename_label = QLabel("Base name:")
        self.filename_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.control_layout.addWidget(self.filename_label)

        self.filename_input = QLineEdit()
        self.filename_input.setPlaceholderText("experiment_name")
        self.filename_input.setStyleSheet("""
            QLineEdit {
                font-size: 14px;
                padding: 8px;
                border: 2px solid #ccc;
                border-radius: 5px;
            }
            QLineEdit:focus {
                border-color: #4CAF50;
            }
        """)
        self.control_layout.addWidget(self.filename_input)

        self.save_button = QPushButton("Save Data")
        self.save_button.setMaximumHeight(50)
        self.save_button.setEnabled(False)
        self.save_button.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        self.save_button.clicked.connect(self.save_all_data)
        self.control_layout.addWidget(self.save_button)

        self.save_shortcut = QShortcut(QKeySequence("Ctrl+S"), self.main_widget)
        self.save_shortcut.activated.connect(self.save_all_data)
        
        self.main_layout.addLayout(self.control_layout)

        # Window
        #self.window = pg.GraphicsLayoutWidget()
        #self.main_layout.addWidget(self.window, stretch=1)

        # Create main window layout using tabs
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #ccc;
                border-radius: 6px;
                background-color: #f5f5f5;
            }
            QTabBar::tab {
                background: #e0e0e0;
                color: #333;
                padding: 10px 20px;
                margin-right: 4px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                font-weight: bold;
                min-width: 150px;
            }
            QTabBar::tab:selected {
                background: #ffffff;
                border-bottom: 2px solid #4CAF50;
                color: #000;
            }
            QTabBar::tab:hover {
                background: #d0d0d0;
            }
        """)
        self.main_layout.addWidget(self.tab_widget, stretch=1)

        # Create a tab of each of the rows. Also add an extra tab for the trayectory plot
        tab1 = QWidget()
        tab1_layout = QVBoxLayout()
        tab1.setLayout(tab1_layout)
        tab1_plots = pg.GraphicsLayoutWidget()
        tab1_plots.setBackground('w')
        tab1_layout.addWidget(tab1_plots)
        self.tab_widget.addTab(tab1, "Position x")

        tab2 = QWidget()
        tab2_layout = QVBoxLayout()
        tab2.setLayout(tab2_layout)
        tab2_plots = pg.GraphicsLayoutWidget()
        tab2_plots.setBackground('w')
        tab2_layout.addWidget(tab2_plots)
        self.tab_widget.addTab(tab2, "Position y")

        tab3 = QWidget()
        tab3_layout = QVBoxLayout()
        tab3.setLayout(tab3_layout)
        tab3_plots = pg.GraphicsLayoutWidget()
        tab3_plots.setBackground('w')
        tab3_layout.addWidget(tab3_plots)
        self.tab_widget.addTab(tab3, "Speed")

        tab4 = QWidget()
        tab4_layout = QVBoxLayout()
        tab4.setLayout(tab4_layout)
        tab4_plots = pg.GraphicsLayoutWidget()
        tab4_plots.setBackground('w')
        tab4_layout.addWidget(tab4_plots)
        self.tab_widget.addTab(tab4, "Acceleration / ERPM")

        tab5 = QWidget()
        tab5_layout = QVBoxLayout()
        tab5.setLayout(tab5_layout)
        tab5_plots = pg.GraphicsLayoutWidget()
        tab5_plots.setBackground('w')
        tab5_layout.addWidget(tab5_plots)
        self.tab_widget.addTab(tab5, "Trayectory")

        # Tab 1

        self.pos_x_plot = tab1_plots.addPlot(row=0, col=0, title="Position X")
        self.pos_x_plot.setLabel('left', 'X', 'm')
        self.pos_x_plot.setLabel('bottom', 'Time', 's')
        self.pos_x_plot.enableAutoRange(axis='x')
        self.pos_x_plot.setLimits(yMin=-4.5, yMax=4.5, xMin=0.0)
        self.pos_x_plot.setYRange(-4.5, 4.5)
        self.pos_x_plot.addLegend()
        self.pos_x_plot.showGrid(x=True, y=True, alpha=0.3)
        self.pos_x_plot.addLine(y=0, pen=pg.mkPen(color='k', width=1, style=QtCore.Qt.DashLine))

        self.error_position_x_plot = tab1_plots.addPlot(row=1, col=0, title="Position X error")
        self.error_position_x_plot.setLabel('left', 'ΔX', 'm')
        self.error_position_x_plot.setLabel('bottom', 'Time', 's')
        self.error_position_x_plot.enableAutoRange(axis='x')
        self.error_position_x_plot.setLimits(yMin=-3.5, yMax=3.5, xMin=0.0)
        self.error_position_x_plot.setYRange(-3.5, 3.5)
        self.error_position_x_plot.showGrid(x=True, y=True, alpha=0.3)
        self.error_position_x_plot.addLine(y=0, pen=pg.mkPen(color='k', width=1, style=QtCore.Qt.DashLine))

        # Tab 2

        self.pos_y_plot = tab2_plots.addPlot(row=0, col=0, title="Position Y")
        self.pos_y_plot.setLabel('left', 'Y', 'm')
        self.pos_y_plot.setLabel('bottom', 'Time', 's')
        self.pos_y_plot.enableAutoRange(axis='x')
        self.pos_y_plot.setLimits(yMin=-4.5, yMax=4.5, xMin=0.0)
        self.pos_y_plot.setYRange(-4.5, 4.5)
        self.pos_y_plot.addLegend()
        self.pos_y_plot.showGrid(x=True, y=True, alpha=0.3)
        self.pos_y_plot.addLine(y=0, pen=pg.mkPen(color='k', width=1, style=QtCore.Qt.DashLine))

        self.error_position_y_plot = tab2_plots.addPlot(row=1, col=0, title="Position Y error")
        self.error_position_y_plot.setLabel('left', 'ΔY', 'm')
        self.error_position_y_plot.setLabel('bottom', 'Time', 's')
        self.error_position_y_plot.enableAutoRange(axis='x')
        self.error_position_y_plot.setLimits(yMin=-3.5, yMax=3.5, xMin=0.0)
        self.error_position_y_plot.setYRange(-3.5, 3.5)
        self.error_position_y_plot.showGrid(x=True, y=True, alpha=0.3)
        self.error_position_y_plot.addLine(y=0, pen=pg.mkPen(color='k', width=1, style=QtCore.Qt.DashLine))
        
        # Tab 3

        self.speed_comparison_plot = tab3_plots.addPlot(row=0, col=0, title="Ref speed vs Real speed")
        self.speed_comparison_plot.setLabel('left', 'Speed', 'm/s')
        self.speed_comparison_plot.setLabel('bottom', 'Time', 's')
        self.speed_comparison_plot.enableAutoRange(axis='x')
        self.speed_comparison_plot.setLimits(yMin=-3.5, yMax=3.5, xMin=0.0)
        self.speed_comparison_plot.setYRange(-3.5, 3.5)
        self.speed_comparison_plot.addLegend()
        self.speed_comparison_plot.showGrid(x=True, y=True, alpha=0.3)
        self.speed_comparison_plot.addLine(y=0, pen=pg.mkPen(color='k', width=1, style=QtCore.Qt.DashLine))

        self.error_speed_plot = tab3_plots.addPlot(row=1, col=0, title="Speed error")
        self.error_speed_plot.setLabel('left', 'Error', 'm/s')
        self.error_speed_plot.setLabel('bottom', 'Time', 's')
        self.error_speed_plot.enableAutoRange(axis='x')
        self.error_speed_plot.setLimits(yMin=-3.5, yMax=3.5, xMin=0.0)
        self.error_speed_plot.setYRange(-3.5, 3.5)
        self.error_speed_plot.showGrid(x=True, y=True, alpha=0.3)
        self.error_speed_plot.addLine(y=0, pen=pg.mkPen(color='k', width=1, style=QtCore.Qt.DashLine))

        # Tab 4

        self.linear_accel_plot = tab4_plots.addPlot(row=0, col=0, title="Linear acceleration")
        self.linear_accel_plot.setLabel('left', 'Accel', 'm/s²')
        self.linear_accel_plot.setLabel('bottom', 'Time', 's')
        self.linear_accel_plot.enableAutoRange(axis='x')
        self.linear_accel_plot.setLimits(yMin=-3.5, yMax=3.5, xMin=0.0)
        self.linear_accel_plot.setYRange(-3.5, 3.5)
        self.linear_accel_plot.addLegend()
        self.linear_accel_plot.showGrid(x=True, y=True, alpha=0.3)
        self.linear_accel_plot.addLine(y=0, pen=pg.mkPen(color='k', width=1, style=QtCore.Qt.DashLine))

        self.erpm_plot = tab4_plots.addPlot(row=1, col=0, title="ERPM")
        self.erpm_plot.setLabel('left', 'ERPM')
        self.erpm_plot.setLabel('bottom', 'Time', 's')
        self.erpm_plot.enableAutoRange(axis='x')
        self.erpm_plot.setLimits(yMin=-23000, yMax=23000, xMin=0.0)
        self.erpm_plot.setYRange(-23000, 23000)
        self.erpm_plot.addLegend()
        self.erpm_plot.showGrid(x=True, y=True, alpha=0.3)
        self.erpm_plot.addLine(y=0, pen=pg.mkPen(color='k', width=1, style=QtCore.Qt.DashLine))

        # Tab 5

        self.trayectory_plot = tab5_plots.addPlot(row=0, col=0, title="Trajectory (X-Y)")
        self.trayectory_plot.setLabel('left', 'Y', 'm')
        self.trayectory_plot.setLabel('bottom', 'X', 'm')
        self.trayectory_plot.setLimits(xMin=-4.5, xMax=4.5, yMin=-4.5, yMax=4.5)
        self.trayectory_plot.setXRange(-4.5, 4.5)
        self.trayectory_plot.setYRange(-4.5, 4.5)
        self.trayectory_plot.addLegend()
        self.trayectory_plot.showGrid(x=True, y=True, alpha=0.3)
        self.trayectory_plot.addLine(y=0, pen=pg.mkPen(color='k', width=1, style=QtCore.Qt.DashLine))
        self.trayectory_plot.addLine(x=0, pen=pg.mkPen(color='k', width=1, style=QtCore.Qt.DashLine))

        # Curves
        self.pos_x_curve = self.pos_x_plot.plot(pen=pg.mkPen(color='g', width=2), name='Odometry')
        self.expected_x_curve = self.pos_x_plot.plot(pen=pg.mkPen(color='m', width=2), name='Expected')
        self.pos_y_curve = self.pos_y_plot.plot(pen=pg.mkPen(color='g', width=2), name='Odometry')
        self.expected_y_curve = self.pos_y_plot.plot(pen=pg.mkPen(color='m', width=2), name='Expected')
        self.trayectory_curve = self.trayectory_plot.plot(pen=pg.mkPen(color='g', width=2), name='Odometry')
        self.expected_traj_curve = self.trayectory_plot.plot(pen=pg.mkPen(color='m', width=2), name='Expected')
        self.speed_odom_curve = self.speed_comparison_plot.plot(pen=pg.mkPen(color='g', width=2), name='Odometry')
        self.speed_input_curve = self.speed_comparison_plot.plot(pen=pg.mkPen(color='m', width=2), name='Input')
        self.linear_accel_x_curve = self.linear_accel_plot.plot(pen=pg.mkPen(color='b', width=2), name='X')
        self.linear_accel_y_curve = self.linear_accel_plot.plot(pen=pg.mkPen(color='r', width=2), name='Y')
        self.error_speed_curve = self.error_speed_plot.plot(pen=pg.mkPen(color='c', width=2), name='Speed error')
        self.erpm_curve = self.erpm_plot.plot(pen=pg.mkPen(color='g', width=2), name='Odometry')
        self.input_erpm_curve = self.erpm_plot.plot(pen=pg.mkPen(color='m', width=2), name='Input')
        self.error_position_x_curve = self.error_position_x_plot.plot(pen=pg.mkPen(color='b', width=2), name='Error')
        self.error_position_y_curve = self.error_position_y_plot.plot(pen=pg.mkPen(color='b', width=2), name='Error')

        # Subscriptions
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.cmd_speed_subscriber = self.create_subscription(Float64, '/commands/motor/speed', self.cmd_speed_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, '/sensors/imu/raw', self.imu_callback, 10)
        self.sensor_subscriber = self.create_subscription(VescStateStamped, '/sensors/core', self.sensor_callback, 10)
        self.expected_x = self.create_subscription(Float64, '/sequencer/expected/x', self.expected_x_callback, 10)
        self.expected_y = self.create_subscription(Float64, '/sequencer/expected/y', self.expected_y_callback, 10)
        self.trajectory_subscriber = self.create_subscription(Float64MultiArray, '/sequencer/trajectory', self.trajectory_callback, 10)

        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)  # 20Hz

        # Parameters
        self.declare_parameter('speed_to_erpm_gain', 4500.0)
        self.speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').value

        # Curve map
        self.curves = {
            'x_odom': self.pos_x_curve,
            'y_odom': self.pos_y_curve,
            'vel': self.speed_odom_curve,
            'input_vel': self.speed_input_curve,
            'x_accel': self.linear_accel_x_curve,
            'y_accel': self.linear_accel_y_curve,
            'erpm': self.erpm_curve,
            'input_erpm': self.input_erpm_curve,
            'vel_error': self.error_speed_curve,
            'exp_x': self.expected_x_curve,
            'exp_y': self.expected_y_curve,
            'x_error': self.error_position_x_curve,
            'y_error': self.error_position_y_curve
        }

        self.get_logger().info('Plotter started â€” waiting for data...')
        self.main_widget.show()

    def get_base_filename(self):

        name = self.filename_input.text().strip()
        if not name:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            name = f"experiment_{timestamp}"

        return name
    
    def save_plots(self, save_dir):

        plots_dir = os.path.join(save_dir, 'plots')
        png_dir = os.path.join(plots_dir, 'png')
        svg_dir = os.path.join(plots_dir, 'svg')
        os.makedirs(png_dir, exist_ok=True)
        os.makedirs(svg_dir, exist_ok=True)

        tab1_plots = self.tab_widget.widget(0).findChild(pg.GraphicsLayoutWidget)
        tab2_plots = self.tab_widget.widget(1).findChild(pg.GraphicsLayoutWidget)
        tab3_plots = self.tab_widget.widget(2).findChild(pg.GraphicsLayoutWidget)
        tab4_plots = self.tab_widget.widget(3).findChild(pg.GraphicsLayoutWidget)
        tab5_plots = self.tab_widget.widget(4).findChild(pg.GraphicsLayoutWidget)
        
        plot_widgets = {
            'position_x': tab1_plots,
            'position_y': tab2_plots,
            'speed': tab3_plots,
            'accel_erpm': tab4_plots,
            'trayectory': tab5_plots
            
        }
        
        for name, widget in plot_widgets.items():
            if widget is not None:
                try:
                    # Save as PNG
                    png_exporter = pg.exporters.ImageExporter(widget.scene())
                    png_exporter.parameters()['width'] = 1920
                    png_exporter.parameters()['height'] = 1080
                    png_exporter.parameters()['antialias'] = True
                    png_exporter.export(os.path.join(png_dir, f'{name}.png'))
                    
                    # Save as SVG
                    svg_exporter = pg.exporters.SVGExporter(widget.scene())
                    svg_exporter.export(os.path.join(svg_dir, f'{name}.svg'))
                    
                    self.get_logger().info(f'Saved plots: {name}.png and {name}.svg')
                except Exception as e:
                    self.get_logger().error(f'Failed to save plot {name}: {e}')
    
    def start_rosbag_recording(self):

        try:
            base_name = self.get_base_filename()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_filename = f"{base_name}_{timestamp}.bag"

            topics = [
                '/odometry/filtered',
                '/drive',
                '/sequencer/expected/x',
                '/sequencer/expected/y',
                '/sequencer/trajectory',
                '/sensors/core',
                '/sensors/imu/raw',
                '/commands/motor/speed'
            ]

            cmd = ['ros2', 'bag', 'record', '-o', bag_filename] + topics
            self.rosbag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            self.get_logger().info(f'Started ROS bag recording: {bag_filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to start ROS bag: {e}')
    
    def stop_rosbag_recording(self):

        if self.rosbag_process:
            self.rosbag_process.send_signal(signal.SIGINT)
            try:
                self.rosbag_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.rosbag_process.kill()
            self.rosbag_process = None
            self.get_logger().info('Stopped ROS bag recording')

    def save_all_data(self):
        with self.lock:
            base_name = self.get_base_filename()
            
            # Create directory for this experiment
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_dir = f"{base_name}_{timestamp}"
            os.makedirs(save_dir, exist_ok=True)
            
            # Save CSV files
            self.save_csv_data(save_dir)
            
            # Save .mat file
            self.save_mat_data(save_dir, base_name)

            # Save png files and svg files
            self.save_plots(save_dir)
            
            # Note ROS bag recording
            if self.rosbag_process:
                self.get_logger().info(f'ROS bag recording in progress: {base_name}.bag')
            
            # Save metadata
            self.save_metadata(save_dir, base_name)
            
            self.get_logger().info(f'All data saved to {save_dir}/')

    def save_csv_data(self, save_dir):

        csv_dir = os.path.join(save_dir, 'csv')
        os.makedirs(csv_dir, exist_ok=True)

        for name, buf in self.data.items():
            if len(buf['ts']) == 0:
                continue

            ts = np.array(buf['ts'], dtype=float)
            val = np.array(buf['val'], dtype=float)

            if self.start_time is not None:
                rel_time = ts - self.start_time
            else:
                rel_time = ts

            filename = os.path.join(csv_dir, f"{name}.csv")
            with open(filename, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['timestamp', 'time_relative', name])
                    for t_abs, t_rel, v in zip(ts, rel_time, val):
                        writer.writerow([t_abs, t_rel, v])
                
            self.get_logger().info(f'Saved {name}.csv ({len(ts)} points)')
                                   
        if len(self.traj_x) > 0:
            traj_data = np.column_stack((
                np.array(self.traj_x, dtype=float),
                np.array(self.traj_y, dtype=float)
            ))
            np.savetxt(
                os.path.join(csv_dir, 'trajectory.csv'),
                traj_data,
                delimiter=',',
                header='X,Y',
                comments=''
            )
        
        if len(self.exp_x) > 0:
            exp_data = np.column_stack((
                np.array(self.exp_x, dtype=float),
                np.array(self.exp_y, dtype=float)
            ))
            np.savetxt(
                os.path.join(csv_dir, 'expected_trajectory.csv'),
                exp_data,
                delimiter=',',
                header='X,Y',
                comments=''
            )

    def save_mat_data(self, save_dir, base_name):

        mat_data = {}

        for name, buf in self.data.items():
            if len(buf['ts']) > 0:
                ts = np.array(buf['ts'], dtype=float)
                val = np.array(buf['val'], dtype=float)
                
                if self.start_time is not None:
                    rel_time = ts - self.start_time
                else:
                    rel_time = ts
                
                mat_data[name] = {
                    'timestamp': ts,
                    'time': rel_time,
                    'value': val
                }

        if len(self.traj_x) > 0:
            mat_data['trajectory'] = np.column_stack((
                np.array(self.traj_x, dtype=float),
                np.array(self.traj_y, dtype=float)
            ))
        
        if len(self.exp_x) > 0:
            mat_data['expected_trajectory'] = np.column_stack((
                np.array(self.exp_x, dtype=float),
                np.array(self.exp_y, dtype=float)
            ))
        
        mat_filename = os.path.join(save_dir, f"{base_name}.mat")
        sio.savemat(mat_filename, mat_data)
        self.get_logger().info(f'Saved {base_name}.mat')


    def save_metadata(self, save_dir, base_name):
        with open(os.path.join(save_dir, 'metadata.txt'), 'w') as f:
            f.write(f"Experiment: {base_name}\n")
            f.write(f"Save time: {datetime.now()}\n")
            f.write(f"Start time: {self.start_time}\n")
            f.write(f"Duration: {datetime.now().timestamp() - self.start_time if self.start_time else 0:.2f}s\n\n")
            
            f.write("Data buffers:\n")
            for name, buf in self.data.items():
                f.write(f"  {name}: {len(buf['ts'])} points\n")
            
            f.write(f"\nTrajectory points: {len(self.traj_x)}\n")
            f.write(f"Expected trajectory points: {len(self.exp_x)}\n")
            
            f.write("\nROS topics recorded:\n")
            f.write("  /odometry/filtered\n")
            f.write("  /drive\n")
            f.write("  /sequencer/expected/x\n")
            f.write("  /sequencer/expected/y\n")
            f.write("  /sequencer/trajectory\n")
            f.write("  /sensors/core\n")
            f.write("  /sensors/imu/raw\n")
            f.write("  /commands/motor/speed\n")
        
    def add_buffer(self, name, maxlen=5000):
        self.data[name] = {
            'ts': deque(maxlen=maxlen),
            'val': deque(maxlen=maxlen)
        }

    def _get_time(self, msg=None):
        if msg is not None and hasattr(msg, 'header'):
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return self.get_clock().now().nanoseconds * 1e-9

    def trajectory_callback(self,msg):
        with self.lock:

            data = msg.data
            if len(data) % 2 != 0:
                self.get_logger().warn('Trajectory data not even')
                return
            
            for i in range (0, len(data), 2):
                x = data[i]
                y = data[i+1]
                self.exp_x.append(x)
                self.exp_y.append(y)
            

    def expected_x_callback(self,msg):
        with self.lock:
            t = self.get_clock().now().nanoseconds * 1e-9

            input_exp_x = msg.data
            
            self.data['exp_x']['ts'].append(t)
            self.data['exp_x']['val'].append(input_exp_x)

    def expected_y_callback(self,msg):
        with self.lock:
            t = self.get_clock().now().nanoseconds * 1e-9

            input_exp_y = msg.data

            self.data['exp_y']['ts'].append(t)
            self.data['exp_y']['val'].append(input_exp_y)

    def odom_callback(self, msg):
        with self.lock:
            t = self._get_time(msg)
            if self.start_time is None:
                self.start_time = t
                self.get_logger().info(f'First data received at t={t:.2f}')
            
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.traj_x.append(x)
            self.traj_y.append(y)

            q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                 msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            yaw = Rotation.from_quat(q).as_euler('xyz', degrees=False)[2]
            v = msg.twist.twist.linear.x

            for name, val in [('x_odom', x), ('y_odom', y), ('yaw_odom', yaw), ('vel', v)]:
                self.data[name]['ts'].append(t)
                self.data[name]['val'].append(val)

    def cmd_speed_callback(self, msg):
        with self.lock:
            t = self._get_time()
            input_erpm = msg.data
            input_vel = input_erpm / self.speed_to_erpm_gain
            self.data['input_erpm']['ts'].append(t)
            self.data['input_erpm']['val'].append(input_erpm)
            self.data['input_vel']['ts'].append(t)
            self.data['input_vel']['val'].append(input_vel)

    def imu_callback(self, msg):
        with self.lock:
            t = self._get_time(msg)
            self.data['x_accel']['ts'].append(t)
            self.data['x_accel']['val'].append(msg.linear_acceleration.x)
            self.data['y_accel']['ts'].append(t)
            self.data['y_accel']['val'].append(msg.linear_acceleration.y)

    def sensor_callback(self, msg):
        with self.lock:
            t = self._get_time(msg)
            self.data['erpm']['ts'].append(t)
            self.data['erpm']['val'].append(msg.state.speed)

    def toggle_plotting(self):
        with self.lock:
            if not self.plotting_active and not self.plotting_paused:
                # START
                self.plotting_active = True
                self.plotting_paused = False
                self.save_button.setEnabled(True)
                self.start_rosbag_recording()
                self.toggle_button.setText("Pause")
                self.toggle_button.setStyleSheet("""
                    QPushButton {
                        background-color: #FF9800;
                        color: white;
                        font-size: 16px;
                        font-weight: bold;
                        padding: 10px;
                        border-radius: 5px;
                    }
                    QPushButton:hover {
                        background-color: #F57C00;
                    }
                """)
                self.get_logger().info('Plotting STARTED')
                
            elif self.plotting_active and not self.plotting_paused:
                # PAUSE
                self.plotting_active = False
                self.plotting_paused = True
                self.stop_rosbag_recording()
                self.toggle_button.setText("Reset")
                self.toggle_button.setStyleSheet("""
                    QPushButton {
                        background-color: #f44336;
                        color: white;
                        font-size: 16px;
                        font-weight: bold;
                        padding: 10px;
                        border-radius: 5px;
                    }
                    QPushButton:hover {
                        background-color: #d32f2f;
                    }
                """)
                self.get_logger().info('Plotting PAUSED')
                
            else:
                # RESET
                self.plotting_active = False
                self.plotting_paused = False
                self.start_time = None
                self.save_button.setEnabled(False)
                
                for name in self.data:
                    self.data[name]['ts'].clear()
                    self.data[name]['val'].clear()
                self.traj_x.clear()
                self.traj_y.clear()
                self.exp_x.clear()
                self.exp_y.clear()
                
                for curve in self.curves.values():
                    curve.clear()
                self.trayectory_curve.clear()
                self.expected_traj_curve.clear()

                tab1_plots = self.tab_widget.widget(0).findChild(pg.GraphicsLayoutWidget)
                tab2_plots = self.tab_widget.widget(1).findChild(pg.GraphicsLayoutWidget)
                tab3_plots = self.tab_widget.widget(2).findChild(pg.GraphicsLayoutWidget)
                tab4_plots = self.tab_widget.widget(3).findChild(pg.GraphicsLayoutWidget)
                tab5_plots = self.tab_widget.widget(4).findChild(pg.GraphicsLayoutWidget)

                for plot_widget in [tab1_plots, tab2_plots, tab3_plots, tab4_plots, tab5_plots]:
                    if plot_widget is not None:
                        plot_widget.update()
                
                self.toggle_button.setText("Start")
                self.toggle_button.setStyleSheet("""
                    QPushButton {
                        background-color: #4CAF50;
                        color: white;
                        font-size: 16px;
                        font-weight: bold;
                        padding: 10px;
                        border-radius: 5px;
                    }
                    QPushButton:hover {
                        background-color: #45a049;
                    }
                """)
                self.get_logger().info('Plotting RESET')

    def update_plots(self):
        if not self.plotting_active:
            return
        try:
            with self.lock:
                if self.start_time is None:
                    return

                snapshots = {}
                for name, buf in self.data.items():
                    if len(buf['ts']) > 0:
                        ts = np.array(buf['ts'], dtype=float) - self.start_time
                        val = np.array(buf['val'], dtype=float)
                        snapshots[name] = (ts, val)

                traj_x = np.array(self.traj_x, dtype=float)
                traj_y = np.array(self.traj_y, dtype=float)

                exp_x = np.array(self.exp_x, dtype=float)
                exp_y = np.array(self.exp_y, dtype=float)

                # Speed error
                if 'vel' in snapshots and 'input_vel' in snapshots:
                    vel_ts, vel_val = snapshots['vel']
                    inp_ts, inp_val = snapshots['input_vel']
                    if len(vel_ts) > 1 and len(inp_ts) > 1:
                        interp_input = np.interp(vel_ts, inp_ts, inp_val)
                        error = interp_input - vel_val
                        snapshots['vel_error'] = (vel_ts, error)

                # Position x error
                if 'x_odom' in snapshots and 'exp_x' in snapshots:
                    x_odom_ts, x_odom_val = snapshots['x_odom']
                    x_exp_ts, x_exp_val = snapshots['exp_x']
                    if len(x_odom_ts) > 1 and len(x_exp_ts) > 1:
                        interp_input = np.interp(x_odom_ts, x_exp_ts, x_exp_val)
                        error = interp_input - x_odom_val
                        snapshots['x_error'] = (x_odom_ts, error)

                # Position y error
                if 'y_odom' in snapshots and 'exp_y' in snapshots:
                    y_odom_ts, y_odom_val = snapshots['y_odom']
                    y_exp_ts, y_exp_val = snapshots['exp_y']
                    if len(y_odom_ts) > 1 and len(y_exp_ts) > 1:
                        interp_input = np.interp(y_odom_ts, y_exp_ts, y_exp_val)
                        error = interp_input - y_odom_val
                        snapshots['y_error'] = (y_odom_ts, error)

            # Update plots
            for name, (x, y) in snapshots.items():
                if name in self.curves and len(x) > 1:
                    self.curves[name].setData(x, y)

            if len(traj_x) > 1:
                self.trayectory_curve.setData(traj_x, traj_y)

            if len(exp_x) > 1 and len(exp_y) > 1:
                self.expected_traj_curve.setData(exp_x, exp_y)

        except Exception as e:
            self.get_logger().error(f'Update error: {e}')

def main():
    app = pg.mkQApp()

    rclpy.init()
    node = PlotterNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    
    app.exec_()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
