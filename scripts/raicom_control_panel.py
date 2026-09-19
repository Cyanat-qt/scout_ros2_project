#!/usr/bin/env python3

import os
import re
import signal
import sys
from pathlib import Path

from PyQt5.QtCore import QProcess
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QPlainTextEdit
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget


ROOT = Path(__file__).resolve().parents[1]
CONTROL_SCRIPT = ROOT / 'scripts' / 'raicom_sim_control.sh'
LOG_DIR = '/tmp/raicom_gui_logs'
CENTERLINE_LOG = f'{LOG_DIR}/centerline.log'
CENTERLINE_LOG_KEYWORDS = (
    'YOLO HTTP detector ready',
    'YOLO activated',
    'YOLO deactivated',
    'YOLO:',
    'YOLO result',
    'YOLO timeout',
    '战区',
    'recognizing_segment=',
    'recognized_segment=',
    'completed_one_lap',
    '[WARN]',
    '[ERROR]',
)


class CommandRunner(QProcess):
    def __init__(self, label, command, parent=None):
        super().__init__(parent)
        self.label = label
        self.command = command


class RaicomControlPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('仿真控制台')
        self.resize(880, 620)
        self.runners = []
        self.centerline_log_runner = None

        self.status_label = QLabel('状态：未查询')
        self.status_label.setObjectName('statusLabel')

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.log_view.setFont(QFont('Monospace', 10))
        self.log_view.setMaximumBlockCount(3000)

        self.buttons = []
        button_specs = [
            ('一键启动仿真', 'start-sim', '启动 RAICOM 场地 Gazebo 仿真'),
            ('停止仿真', 'stop-sim', '停止 Gazebo/导航 launch，保留容器'),
            ('停止所有节点', 'stop-nodes', '停止容器内 ROS/Gazebo/RViz 进程'),
            ('重启所有节点', 'restart-nodes', '重启默认仿真节点'),
            ('打开 RViz', 'open-rviz', '打开 navigation.rviz'),
            ('关闭 RViz', 'close-rviz', '关闭 RViz 进程'),
            ('URDF 模型 RViz', 'urdf-rviz', '只在 RViz 中显示当前仿真使用的机器人 URDF 模型'),
            ('Gazebo 地图查看', 'gazebo-map', '只打开 Gazebo 场地地图，不启动机器人和导航'),
            ('启动底盘测试', 'chassis-test', '开放场地直行、左平移、顺时针 180 度测试'),
            ('中心线规划器', 'centerline-plan', '在地图上绘制横平竖直中心线并保存配置'),
            ('中心线单圈测试', 'centerline-lap', '启动全向中心线控制器单圈测试'),
            ('赛事单圈导航', 'competition-lap', '打开 Gazebo、相机画面和 YOLO 识别，执行赛事单圈导航'),
            ('中心线参数自调', 'centerline-tune', '无 GUI 仿真搜索速度和平滑减速参数'),
            ('纯 Nav2 单圈兜底', 'nav-lap', '启动 Nav2 MPPI 并运行 101,102,103 单圈路线'),
        ]

        grid = QGridLayout()
        for index, (text, command, tooltip) in enumerate(button_specs):
            button = QPushButton(text)
            button.setToolTip(tooltip)
            button.setMinimumHeight(44)
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            button.clicked.connect(lambda checked=False, c=command, t=text: self.run_command(t, c))
            grid.addWidget(button, index // 2, index % 2)
            self.buttons.append(button)

        status_button = QPushButton('刷新状态')
        status_button.clicked.connect(lambda: self.run_command('刷新状态', 'status', quiet=True))

        clear_button = QPushButton('清空日志')
        clear_button.clicked.connect(self.log_view.clear)

        bottom = QHBoxLayout()
        bottom.addWidget(status_button)
        bottom.addWidget(clear_button)
        bottom.addStretch(1)

        layout = QVBoxLayout()
        layout.addWidget(self.status_label)
        layout.addLayout(grid)
        layout.addWidget(self.log_view, stretch=1)
        layout.addLayout(bottom)

        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)
        self.apply_style()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(lambda: self.run_command('刷新状态', 'status', quiet=True))
        self.status_timer.start(5000)
        QTimer.singleShot(200, lambda: self.run_command('刷新状态', 'status', quiet=True))

    def apply_style(self):
        self.setStyleSheet(
            '''
            QMainWindow { background: #f5f6f8; }
            QLabel#statusLabel {
                padding: 10px 12px;
                background: #ffffff;
                border: 1px solid #d9dee7;
                border-radius: 6px;
                color: #1f2937;
            }
            QPushButton {
                background: #ffffff;
                border: 1px solid #cfd6e2;
                border-radius: 6px;
                padding: 8px 12px;
                color: #111827;
                font-size: 14px;
            }
            QPushButton:hover { background: #eef4ff; border-color: #9bb7e8; }
            QPushButton:pressed { background: #dbeafe; }
            QPushButton:disabled { color: #8a94a6; background: #f0f2f5; }
            QPlainTextEdit {
                background: #111827;
                color: #e5e7eb;
                border: 1px solid #202938;
                border-radius: 6px;
                padding: 8px;
            }
            '''
        )

    def append_log(self, text):
        if not text:
            return
        cursor = self.log_view.textCursor()
        cursor.movePosition(cursor.End)
        cursor.insertText(text)
        self.log_view.setTextCursor(cursor)
        self.log_view.ensureCursorVisible()

    def container_name(self):
        return os.environ.get('RAICOM_CONTAINER', 'raicom-sim')

    def clean_ros_log_line(self, line):
        line = line.strip()
        line = re.sub(r'^\[[^\]]+\]\s+', '', line)
        line = re.sub(r'^\[[^\]]+\]\s+\[[^\]]+\]\s+\[[^\]]+\]:\s*', '', line)
        line = re.sub(r'^\[[^\]]+\]\s+', '', line)
        return line

    def stop_centerline_log_follow(self):
        runner = self.centerline_log_runner
        self.centerline_log_runner = None
        if runner is None:
            return
        if runner.state() != QProcess.NotRunning:
            runner.terminate()
            if not runner.waitForFinished(800):
                runner.kill()
        runner.deleteLater()

    def start_centerline_log_follow(self):
        self.stop_centerline_log_follow()

        runner = QProcess(self)
        runner.setProgram('docker')
        runner.setArguments([
            'exec',
            self.container_name(),
            'bash',
            '-lc',
            f'mkdir -p {LOG_DIR}; touch {CENTERLINE_LOG}; tail -n 20 -F {CENTERLINE_LOG}',
        ])
        runner.readyReadStandardOutput.connect(self.read_centerline_log)
        runner.readyReadStandardError.connect(self.read_centerline_log_error)
        runner.finished.connect(lambda _code, _status: self.centerline_log_finished(runner))
        self.centerline_log_runner = runner
        self.append_log(f'[中心线日志] 正在输出关键日志：{CENTERLINE_LOG}\n')
        runner.start()

    def read_centerline_log(self):
        runner = self.centerline_log_runner
        if runner is None:
            return
        text = bytes(runner.readAllStandardOutput()).decode(errors='replace')
        for raw_line in text.splitlines():
            if not any(keyword in raw_line for keyword in CENTERLINE_LOG_KEYWORDS):
                continue
            if 'aligning_recognition_segment=' in raw_line and 'yaw_error=' in raw_line:
                continue
            line = self.clean_ros_log_line(raw_line)
            if line:
                self.append_log(f'[中心线] {line}\n')

    def read_centerline_log_error(self):
        runner = self.centerline_log_runner
        if runner is None:
            return
        text = bytes(runner.readAllStandardError()).decode(errors='replace')
        if text.strip():
            self.append_log(text)

    def centerline_log_finished(self, runner):
        if self.centerline_log_runner is runner:
            self.centerline_log_runner = None

    def set_buttons_enabled(self, enabled):
        for button in self.buttons:
            button.setEnabled(enabled)

    def run_command(self, label, command, quiet=False):
        if not CONTROL_SCRIPT.exists():
            QMessageBox.critical(self, '缺少控制脚本', f'找不到 {CONTROL_SCRIPT}')
            return

        if command in (
            'stop-nodes',
            'stop-sim',
            'restart-nodes',
            'chassis-test',
            'centerline-lap',
            'competition-lap',
            'nav-lap',
            'urdf-rviz',
            'gazebo-map',
        ):
            self.stop_centerline_log_follow()

        if not quiet:
            self.append_log(f'\n$ {CONTROL_SCRIPT} {command}\n')
            self.status_label.setText(f'状态：正在执行 {label}')

        runner = CommandRunner(label, command, self)
        env = runner.processEnvironment()
        env.insert('RAICOM_ROOT', str(ROOT))
        env.insert('RAICOM_WS', str(ROOT / 'ros2_ws'))
        env.insert('RAICOM_CONTAINER', self.container_name())
        env.insert('RAICOM_DOCKER_IMAGE', os.environ.get('RAICOM_DOCKER_IMAGE', 'raicom-humble-x11:compiled'))
        runner.setProcessEnvironment(env)
        runner.setWorkingDirectory(str(ROOT))
        runner.setProgram(str(CONTROL_SCRIPT))
        runner.setArguments([command])
        runner.readyReadStandardOutput.connect(lambda r=runner: self.read_output(r, quiet))
        runner.readyReadStandardError.connect(lambda r=runner: self.read_error(r, quiet))
        runner.finished.connect(lambda code, status, r=runner, q=quiet: self.command_finished(r, code, q))

        self.runners.append(runner)
        if not quiet:
            self.set_buttons_enabled(False)
        runner.start()

    def read_output(self, runner, quiet):
        text = bytes(runner.readAllStandardOutput()).decode(errors='replace')
        if runner.command == 'status':
            first_line = text.strip().splitlines()[0] if text.strip() else '无状态输出'
            self.status_label.setText(f'状态：{first_line}')
        if not quiet:
            self.append_log(text)

    def read_error(self, runner, quiet):
        text = bytes(runner.readAllStandardError()).decode(errors='replace')
        if not quiet:
            self.append_log(text)

    def command_finished(self, runner, exit_code, quiet):
        if runner in self.runners:
            self.runners.remove(runner)
        if not quiet:
            self.append_log(f'[{runner.label}] exit={exit_code}\n')
            self.status_label.setText(f'状态：{runner.label} 完成，退出码 {exit_code}')
            self.set_buttons_enabled(True)
        if runner.command in ('centerline-lap', 'competition-lap') and exit_code == 0:
            self.start_centerline_log_follow()
        runner.deleteLater()

    def closeEvent(self, event):
        self.stop_centerline_log_follow()
        for runner in list(self.runners):
            if runner.state() != QProcess.NotRunning:
                runner.terminate()
                if not runner.waitForFinished(800):
                    runner.kill()
        event.accept()


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication(sys.argv)
    window = RaicomControlPanel()
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
