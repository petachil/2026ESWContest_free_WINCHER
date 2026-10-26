import sys
import socket
import struct
import time
import datetime
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QLabel, QPushButton, 
                             QDoubleSpinBox, QGroupBox, QCheckBox, QDialog)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import pyqtgraph as pg

# Matplotlib 및 plt 추가 
import matplotlib.pyplot as plt 
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

# ==========================================
# 1. Network UDP Receiver Thread
# ==========================================
class UDPReceiver(QThread):
    data_received = pyqtSignal(dict) 

    def __init__(self, port=12345):
        super().__init__()
        self.port = port
        self.running = True 
        
    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        sock.bind(('0.0.0.0', self.port))
        sock.settimeout(0.1)

        pkt_format = "<B I f f f f f f f B f B" 
        pkt_size = struct.calcsize(pkt_format)

        while self.running:
            try:
                data, _ = sock.recvfrom(1024)
                if len(data) == pkt_size:
                    unpacked = struct.unpack(pkt_format, data)
                    payload = {
                        "winch_id": unpacked[0],
                        "ts": unpacked[1],
                        "roll": unpacked[2],
                        "pitch": unpacked[3],
                        "gyroX": unpacked[4],
                        "gyroY": unpacked[5],
                        "theta_mag": unpacked[6],
                        "length": unpacked[7],
                        "L_dot": unpacked[8],
                        "smc_state": unpacked[9],
                        "tinyml_comp": unpacked[10],
                        "sd_status": unpacked[11]
                    }
                    self.data_received.emit(payload)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"UDP Recv Error: {e}")

        sock.close()

    def stop(self):
        self.running = False
        self.wait()

# ==========================================
# 2. Individual Winch Control Panel Widget
# ==========================================
class WinchPanel(QWidget):
    def __init__(self, winch_id, parent_gui):
        super().__init__()
        self.w_id = winch_id
        self.gui = parent_gui
        self.initial_synced = False 
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(3, 5, 3, 5)
        layout.setSpacing(8) 

        self.setStyleSheet("QGroupBox { font-weight: bold; }")

        if self.w_id == 1:
            title_html = (
                "<div style='font-size: 24px; font-weight: bold; color: red; "
                "text-align: center; line-height: 110%; padding: 4px 0px;'>"
                "WINCH 1<br><span style='font-size: 18px;'>(CONTROL)</span></div>"
            )
        else:
            title_html = (
                "<div style='font-size: 24px; font-weight: bold; color: #00BFFF; "
                "text-align: center; line-height: 110%; padding: 4px 0px;'>"
                "WINCH 2<br><span style='font-size: 18px;'>(UNCONTROLLED)</span></div>"
            )
        
        title = QLabel(title_html)
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        st_box = QGroupBox("Telemetry")
        st_lay = QGridLayout(st_box)
        self.lbl_net = QLabel("WAITING...")
        self.lbl_net.setStyleSheet("color: blue; font-weight:bold;")
        self.lbl_state = QLabel("WAITING...")
        self.lbl_rp = QLabel("0.0 / 0.0")
        self.lbl_th = QLabel("0.00")
        self.lbl_len = QLabel("0.015")
        self.lbl_spd = QLabel("0.00")
        self.lbl_sd_stat = QLabel("IDLE")
        
        st_lay.addWidget(QLabel("Net:"), 0, 0); st_lay.addWidget(self.lbl_net, 0, 1)
        st_lay.addWidget(QLabel("State:"), 1, 0); st_lay.addWidget(self.lbl_state, 1, 1)
        st_lay.addWidget(QLabel("Roll/Pitch:"), 2, 0); st_lay.addWidget(self.lbl_rp, 2, 1)
        st_lay.addWidget(QLabel("Theta(°):"), 3, 0); st_lay.addWidget(self.lbl_th, 3, 1)
        st_lay.addWidget(QLabel("Length(m):"), 4, 0); st_lay.addWidget(self.lbl_len, 4, 1)
        st_lay.addWidget(QLabel("Speed(cm/s):"), 5, 0); st_lay.addWidget(self.lbl_spd, 5, 1)
        st_lay.addWidget(QLabel("SD Status:"), 6, 0); st_lay.addWidget(self.lbl_sd_stat, 6, 1)
        layout.addWidget(st_box)

        smc_box = QGroupBox("Ctrl Setup")
        smc_lay = QVBoxLayout(smc_box)
        self.btn_smc = QPushButton("Ctrl: OFF")
        self.btn_smc.setCheckable(True); self.btn_smc.setChecked(False)
        self.btn_smc.setStyleSheet("background-color: #9E9E9E; color: black; font-weight: bold;")
        self.btn_smc.clicked.connect(self.send_smc)
        smc_lay.addWidget(self.btn_smc)

        hb = QHBoxLayout()
        self.spin_smc = QDoubleSpinBox()
        self.spin_smc.setRange(1.0, 20.0); self.spin_smc.setValue(6.0); self.spin_smc.setSingleStep(0.5)
        btn_upd = QPushButton("Update")
        btn_upd.clicked.connect(self.send_smc)
        hb.addWidget(QLabel("Tol. Ang(°):")); hb.addWidget(self.spin_smc); hb.addWidget(btn_upd)
        smc_lay.addLayout(hb)
        layout.addWidget(smc_box)

        sd_box = QGroupBox("SD Logging")
        sd_lay = QVBoxLayout(sd_box)
        self.lbl_sd_file = QLabel("Log: Waiting...")
        self.lbl_sd_file.setStyleSheet("color: #607D8B; font-size: 11px;")
        sd_lay.addWidget(self.lbl_sd_file)

        sd_btn_lay = QHBoxLayout()
        b_sd_start = QPushButton("Start Log")
        b_sd_start.setStyleSheet("background-color: #008CBA; color: white;")
        b_sd_start.clicked.connect(self.send_sd_start)
        b_sd_stop = QPushButton("Stop Log")
        b_sd_stop.setStyleSheet("background-color: #f44336; color: white;")
        b_sd_stop.clicked.connect(self.send_sd_stop)
        sd_btn_lay.addWidget(b_sd_start); sd_btn_lay.addWidget(b_sd_stop)
        sd_lay.addLayout(sd_btn_lay)
        layout.addWidget(sd_box)

        cal_box = QGroupBox("Calibration")
        cal_lay = QVBoxLayout(cal_box)
        b_z = QPushButton("Zero IMU (Unlock)")
        b_z.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;")
        b_z.clicked.connect(lambda: self.gui.send_cmd(self.w_id, 1))
        cal_lay.addWidget(b_z)
        layout.addWidget(cal_box)

        jog_box = QGroupBox("Jog Control")
        jog_lay = QVBoxLayout(jog_box)
        hb2 = QHBoxLayout()
        self.spin_jog = QDoubleSpinBox()
        self.spin_jog.setRange(0.1, 2.22); self.spin_jog.setValue(0.5); self.spin_jog.setSingleStep(0.1)
        hb2.addWidget(QLabel("Jog Spd:")); hb2.addWidget(self.spin_jog)
        jog_lay.addLayout(hb2)
        
        self.chk_ovr = QCheckBox("Unlock 1.5cm Limit"); self.chk_ovr.setStyleSheet("color: red;")
        jog_lay.addWidget(self.chk_ovr)

        hb3 = QHBoxLayout()
        b_up = QPushButton("UP (-)"); b_up.clicked.connect(lambda: self.send_jog(-1))
        b_dn = QPushButton("DOWN (+)"); b_dn.clicked.connect(lambda: self.send_jog(1))
        b_st = QPushButton("STOP"); b_st.clicked.connect(lambda: self.send_jog(0))
        hb3.addWidget(b_up); hb3.addWidget(b_dn); hb3.addWidget(b_st)
        jog_lay.addLayout(hb3)
        
        b_lz = QPushButton("Set 1.5cm Length"); b_lz.setStyleSheet("background-color: #9C27B0; color: white;")
        b_lz.clicked.connect(self.send_len_zero)
        jog_lay.addWidget(b_lz)
        layout.addWidget(jog_box)

        auto_box = QGroupBox("Auto Mission")
        auto_lay = QGridLayout(auto_box)
        self.sp_tar = QDoubleSpinBox(); self.sp_tar.setRange(0.05, 1.0); self.sp_tar.setValue(0.7)
        self.sp_spd = QDoubleSpinBox(); self.sp_spd.setRange(0.1, 2.22); self.sp_spd.setValue(1.5)
        self.sp_hov = QDoubleSpinBox(); self.sp_hov.setRange(1.0, 60.0); self.sp_hov.setValue(3.0)
        
        auto_lay.addWidget(QLabel("Target(m):"), 0, 0); auto_lay.addWidget(self.sp_tar, 0, 1)
        auto_lay.addWidget(QLabel("Spd(cm/s):"), 1, 0); auto_lay.addWidget(self.sp_spd, 1, 1)
        auto_lay.addWidget(QLabel("Hover(s):"), 2, 0); auto_lay.addWidget(self.sp_hov, 2, 1)
        
        b_as = QPushButton("Auto Start"); b_as.setStyleSheet("background-color: #4CAF50; color: white;")
        b_as.clicked.connect(self.send_auto)
        b_ax = QPushButton("Auto Stop"); b_ax.setStyleSheet("background-color: #E91E63; color: white;")
        b_ax.clicked.connect(lambda: self.gui.send_cmd(self.w_id, 6))
        auto_lay.addWidget(b_as, 3, 0); auto_lay.addWidget(b_ax, 3, 1)
        layout.addWidget(auto_box)

        layout.addStretch()

    def send_smc(self):
        if self.btn_smc.isChecked():
            self.btn_smc.setText("Ctrl: ON"); self.btn_smc.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
            ang = self.spin_smc.value()
        else:
            self.btn_smc.setText("Ctrl: OFF"); self.btn_smc.setStyleSheet("background-color: #9E9E9E; color: black; font-weight: bold;")
            ang = 90.0
        for _ in range(3):
            self.gui.send_cmd(self.w_id, 7, ang)
            time.sleep(0.005)

    def send_sd_start(self):
        now = datetime.datetime.now()
        yymmdd = float(now.strftime("%y%m%d"))
        hhmm = float(now.strftime("%H%M"))
        self.lbl_sd_file.setText(f"Rec: {now.strftime('%H%M%S')}")
        self.lbl_sd_file.setStyleSheet("color: blue; font-weight: bold; font-size: 11px;")
        self.gui.send_cmd(self.w_id, 9, 1.0, yymmdd, hhmm)

    def send_sd_stop(self):
        self.lbl_sd_file.setText("Log: Saved")
        self.lbl_sd_file.setStyleSheet("color: green; font-weight: bold; font-size: 11px;")
        self.gui.send_cmd(self.w_id, 9, 0.0)

    def send_jog(self, dir_mult):
        spd = abs(self.spin_jog.value() * 0.01) * dir_mult
        ovr = 1.0 if self.chk_ovr.isChecked() else 0.0
        self.gui.send_cmd(self.w_id, 3, spd, ovr)
        
    def send_len_zero(self):
        self.gui.send_cmd(self.w_id, 8)
        self.chk_ovr.setChecked(False)

    def send_auto(self):
        self.gui.last_auto_start_time = time.time() # 개별 자동 미션 시작 시간 기록
        self.gui.send_cmd(self.w_id, 4, self.sp_tar.value(), self.sp_spd.value() * 0.01, self.sp_hov.value())

    def update_ui(self, d):
        self.lbl_net.setText("CONNECTED"); self.lbl_net.setStyleSheet("color: green; font-weight:bold;")
        
        if not self.initial_synced:
            self.send_smc()
            self.initial_synced = True

        if d['length'] <= 0.015 and self.btn_smc.isChecked():
            self.btn_smc.setChecked(False)
            self.send_smc()

        self.lbl_rp.setText(f"{d['roll']*57.3:.1f} / {d['pitch']*57.3:.1f}")
        self.lbl_th.setText(f"{d['theta_mag']*57.3:.2f}")
        self.lbl_len.setText(f"{d['length']:.3f}")
        self.lbl_spd.setText(f"{d['L_dot']*100:.2f}")

        sd_stat = d.get('sd_status', 0)
        if sd_stat == 1:
            self.lbl_sd_stat.setText("LOGGING...")
            self.lbl_sd_stat.setStyleSheet("color: green; font-weight:bold;")
        elif sd_stat == 2:
            self.lbl_sd_stat.setText("SD ERROR")
            self.lbl_sd_stat.setStyleSheet("color: red; font-weight:bold;")
        else:
            self.lbl_sd_stat.setText("IDLE")
            self.lbl_sd_stat.setStyleSheet("color: gray;")

        if d['smc_state'] == 3:
            self.lbl_state.setText("E-STOP!"); self.lbl_state.setStyleSheet("color: white; background-color: red;")
        elif d['smc_state'] == 2:
            self.lbl_state.setText("LOCKED (Zero req)"); self.lbl_state.setStyleSheet("color: orange;")
        elif d['smc_state'] == 1:
            self.lbl_state.setText("ACTIVE (Ctrl)"); self.lbl_state.setStyleSheet("color: red;")
        else:
            self.lbl_state.setText("SAFE (Normal)"); self.lbl_state.setStyleSheet("color: green;")

# ==========================================
# 3. Main GUI Window
# ==========================================
class WinchGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dual Winch Control Dashboard")
        self.resize(1350, 950)

        self.cmd_port = 12346
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.last_auto_start_time = 0.0 # 자동 미션 실행 타임스탬프 저장 변수

        self.max_pts = 200
        self.rt_data = {
            1: {'time': [], 'theta': [], 'length': []},
            2: {'time': [], 'theta': [], 'length': []}
        }
        self.full_data = {
            1: {'time': [], 'length': [], 'theta': [], 'L_dot': [], 'smc': []},
            2: {'time': [], 'length': [], 'theta': [], 'L_dot': [], 'smc': []}
        }

        self.init_ui()

        self.rx_thread = UDPReceiver()
        self.rx_thread.data_received.connect(self.update_telemetry)
        self.rx_thread.start()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        left_panel = QVBoxLayout()
        main_layout.addLayout(left_panel, 1)

        btn_estop = QPushButton("🚨 MASTER E-STOP (BOTH) 🚨")
        btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 18px; height: 50px;")
        btn_estop.clicked.connect(lambda: self.send_cmd(0, 5))
        left_panel.addWidget(btn_estop)
        
        panels_layout = QHBoxLayout()
        self.p1 = WinchPanel(1, self)
        self.p2 = WinchPanel(2, self)
        panels_layout.addWidget(self.p1)
        panels_layout.addWidget(self.p2)
        left_panel.addLayout(panels_layout)

        sim_box = QGroupBox("Simultaneous Auto Mission (Both Winches)")
        sim_box.setStyleSheet("QGroupBox { font-weight: bold; }")
        sim_lay = QGridLayout(sim_box)

        self.sim_sp_tar = QDoubleSpinBox(); self.sim_sp_tar.setRange(0.05, 1.0); self.sim_sp_tar.setValue(0.7)
        self.sim_sp_spd = QDoubleSpinBox(); self.sim_sp_spd.setRange(0.1, 2.22); self.sim_sp_spd.setValue(1.5)
        self.sim_sp_hov = QDoubleSpinBox(); self.sim_sp_hov.setRange(1.0, 60.0); self.sim_sp_hov.setValue(3.0)

        sim_lay.addWidget(QLabel("Target(m):"), 0, 0); sim_lay.addWidget(self.sim_sp_tar, 0, 1)
        sim_lay.addWidget(QLabel("Spd(cm/s):"), 0, 2); sim_lay.addWidget(self.sim_sp_spd, 0, 3)
        sim_lay.addWidget(QLabel("Hover(s):"), 0, 4); sim_lay.addWidget(self.sim_sp_hov, 0, 5)

        b_sim_start = QPushButton("Simultaneous Auto Start")
        b_sim_start.setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold; height: 35px;")
        b_sim_start.clicked.connect(self.send_simultaneous_auto_start)

        b_sim_stop = QPushButton("Simultaneous Auto Stop")
        b_sim_stop.setStyleSheet("background-color: #C2185B; color: white; font-weight: bold; height: 35px;")
        b_sim_stop.clicked.connect(self.send_simultaneous_auto_stop)

        sim_lay.addWidget(b_sim_start, 1, 0, 1, 3)
        sim_lay.addWidget(b_sim_stop, 1, 3, 1, 3)
        left_panel.addWidget(sim_box)

        btn_analyze = QPushButton("Analyze Data")
        btn_analyze.setStyleSheet("background-color: #3F51B5; color: white; font-weight: bold; font-size: 16px; height: 45px;")
        btn_analyze.clicked.connect(self.open_analysis_popup)
        left_panel.addWidget(btn_analyze)

        right_panel = QVBoxLayout()
        main_layout.addLayout(right_panel, 2)

        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')

        title_theta_html = (
            "<b>Pendulum Angle</b> &nbsp;&nbsp;"
            "[<span style='color:red; font-weight:bold;'>Winch1 (Ctrl)</span> | "
            "<span style='color:#00BFFF; font-weight:bold;'>Winch2 (Unctrl)</span>]"
        )
        self.plot_theta = pg.PlotWidget(title=title_theta_html)
        self.plot_theta.showGrid(x=True, y=True)
        self.plot_theta.setYRange(0, 30, padding=0)
        self.plot_theta.setMouseEnabled(y=False)
        self.curve_th1 = self.plot_theta.plot(pen=pg.mkPen('r', width=2))
        self.curve_th2 = self.plot_theta.plot(pen=pg.mkPen((0, 191, 255), width=2))
        right_panel.addWidget(self.plot_theta, stretch=2)

        title_len_html = (
            "<b>Winch Line Length</b> &nbsp;&nbsp;"
            "[<span style='color:red; font-weight:bold;'>Winch1 (Ctrl)</span> | "
            "<span style='color:#00BFFF; font-weight:bold;'>Winch2 (Unctrl)</span>]"
        )
        self.plot_length = pg.PlotWidget(title=title_len_html)
        self.plot_length.showGrid(x=True, y=True)
        self.curve_len1 = self.plot_length.plot(pen=pg.mkPen('r', width=2))
        self.curve_len2 = self.plot_length.plot(pen=pg.mkPen((0, 191, 255), width=2))
        right_panel.addWidget(self.plot_length, stretch=1)

    def send_simultaneous_auto_start(self):
        self.last_auto_start_time = time.time() # 동시 자동 미션 시작 시점 기록
        tar = self.sim_sp_tar.value()
        spd = self.sim_sp_spd.value() * 0.01
        hov = self.sim_sp_hov.value()
        self.send_cmd(0, 4, tar, spd, hov)

    def send_simultaneous_auto_stop(self):
        self.send_cmd(0, 6)

    def send_cmd(self, target_id, cmd_type, p1=0.0, p2=0.0, p3=0.0):
        try:
            pkt = struct.pack("<B B f f f", int(target_id), int(cmd_type), float(p1), float(p2), float(p3))
            self.tx_sock.sendto(pkt, ("192.168.137.255", self.cmd_port))
        except OSError as e:
            print(f"Network Send Error: {e}")

    def update_telemetry(self, d):
        wid = d['winch_id']
        if wid not in [1, 2]: return

        if wid == 1: self.p1.update_ui(d)
        else: self.p2.update_ui(d)

        now = time.time()
        theta_deg = d['theta_mag'] * 57.2958
        
        self.full_data[wid]['time'].append(now)
        self.full_data[wid]['length'].append(d['length'])
        self.full_data[wid]['theta'].append(theta_deg)
        self.full_data[wid]['L_dot'].append(d['L_dot'])
        self.full_data[wid]['smc'].append(d['smc_state'])

        self.rt_data[wid]['time'].append(now)
        self.rt_data[wid]['theta'].append(theta_deg)
        self.rt_data[wid]['length'].append(d['length'])

        if len(self.rt_data[wid]['time']) > self.max_pts:
            self.rt_data[wid]['time'].pop(0)
            self.rt_data[wid]['theta'].pop(0)
            self.rt_data[wid]['length'].pop(0)

        base_t = min((self.rt_data[1]['time'][0] if self.rt_data[1]['time'] else now),
                     (self.rt_data[2]['time'][0] if self.rt_data[2]['time'] else now))
        
        t1 = [t - base_t for t in self.rt_data[1]['time']]
        t2 = [t - base_t for t in self.rt_data[2]['time']]

        self.curve_th1.setData(t1, self.rt_data[1]['theta'])
        self.curve_len1.setData(t1, self.rt_data[1]['length'])
        self.curve_th2.setData(t2, self.rt_data[2]['theta'])
        self.curve_len2.setData(t2, self.rt_data[2]['length'])

    def open_analysis_popup(self):
        popup = AnalysisDialog(self.full_data, self.last_auto_start_time, self)
        popup.exec_()

    def closeEvent(self, event):
        self.rx_thread.stop()
        event.accept()

# ==========================================
# 4. Data Analysis Popup (Matplotlib)
# ==========================================
class AnalysisDialog(QDialog):
    def __init__(self, full_data, auto_start_time=0.0, parent=None):
        super().__init__(parent)
        self.auto_start_time = auto_start_time # Auto Start 버튼 클릭 시점 수신
        self.setWindowTitle("Data Analysis Dashboard (1.5cm ~ 1.5cm Mission)")
        self.setWindowFlags(self.windowFlags() | Qt.WindowMaximizeButtonHint | Qt.WindowMinimizeButtonHint)
        self.resize(1200, 950)
        layout = QVBoxLayout(self)

        self.fig = Figure(figsize=(11, 9), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        self.toolbar = NavigationToolbar(self.canvas, self)

        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)

        self.plot_analysis(full_data)

    def calc_cum_dist(self, lengths):
        if len(lengths) == 0: return np.array([])
        l_arr = np.array(lengths)
        diffs = np.abs(np.diff(l_arr))
        return np.insert(np.cumsum(diffs), 0, 0) + l_arr[0]

    # [핵심 수정] Auto Start 시점 이후 데이터만 탐색하여 수동조작(Jog) 오작동 방지
    def get_mission_mask(self, times, lengths):
        t_arr = np.array(times)
        lens = np.array(lengths)
        if len(lens) < 2 or len(t_arr) < 2:
            return np.zeros_like(lens, dtype=bool)

        # 1) Auto Start 버튼을 누른 시점(last_auto_start_time) 이후 데이터 위치 필터링
        if self.auto_start_time > 0:
            post_start_indices = np.where(t_arr >= (self.auto_start_time - 0.5))[0]
            offset = post_start_indices[0] if len(post_start_indices) > 0 else 0
        else:
            offset = 0

        lens_search = lens[offset:]
        if len(lens_search) == 0:
            return np.zeros_like(lens, dtype=bool)

        # 2) Auto Start 이후 1.5cm(0.016m)를 최초로 벗어나 하강하기 시작하는 시점
        start_rel = np.where(lens_search > 0.016)[0]
        if len(start_rel) == 0:
            return np.zeros_like(lens, dtype=bool)
        
        start_idx = offset + start_rel[0]
        if start_idx > 0:
            start_idx -= 1 # 출발 순간(1.5cm) 포함

        # 3) 하강 후(최소 5cm 이상 하강) 다시 1.5cm(0.0155m 이하)로 복귀 완료한 첫 시점
        deep_indices = np.where(lens[start_idx:] > 0.045)[0]
        if len(deep_indices) > 0:
            after_descent_idx = start_idx + deep_indices[0]
            return_indices = np.where(lens[after_descent_idx:] <= 0.0155)[0]
            if len(return_indices) > 0:
                end_idx = after_descent_idx + return_indices[0] + 1
            else:
                end_idx = len(lens)
        else:
            end_idx = len(lens)

        mask = np.zeros_like(lens, dtype=bool)
        mask[start_idx:end_idx] = True
        return mask

    def compute_metrics(self, data_dict):
        lens = np.array(data_dict['length'])
        th = np.array(data_dict['theta'])       
        times = np.array(data_dict['time'])     
        ddot_sub = np.array(data_dict.get('L_dot', []))

        if len(lens) < 2 or len(times) < 2:
            return 0.0, 0.0, 0.0, 0.0, 0.0

        # Auto Start 이후 1.5cm -> 1.5cm 미션 구간 마스크 적용
        mask = self.get_mission_mask(times, lens)
        
        if not np.any(mask):
            return 0.0, 0.0, 0.0, 0.0, 0.0

        th_sub = th[mask]
        t_sub = times[mask]
        ddot_sub = ddot_sub[mask] if len(ddot_sub) == len(lens) else np.zeros(len(th_sub))

        if len(t_sub) > 1:
            dt = np.diff(t_sub, prepend=t_sub[0])
            dt[0] = dt[1] if len(dt) > 1 else 0.01
            dt = np.clip(dt, 0.001, 0.1)
        else:
            dt = np.array([0.01])

        rmse = np.sqrt(np.mean(th_sub ** 2)) if len(th_sub) > 0 else 0.0
        peak = np.max(np.abs(th_sub)) if len(th_sub) > 0 else 0.0
        sway_energy = np.sum((th_sub ** 2) * dt)
        ctrl_energy = np.sum((ddot_sub ** 2) * dt) if len(ddot_sub) > 0 else 0.0
        total_time = float(t_sub[-1] - t_sub[0]) if len(t_sub) > 1 else 0.0

        return rmse, peak, sway_energy, ctrl_energy, total_time

    def plot_analysis(self, full_data):
        self.fig.clear()
        
        gs = self.fig.add_gridspec(4, 1, height_ratios=[2.5, 0.6, 2.5, 2.5])

        ax1 = self.fig.add_subplot(gs[0])
        ax2 = self.fig.add_subplot(gs[1], sharex=ax1)
        ax3 = self.fig.add_subplot(gs[2], sharex=ax1)
        ax_table = self.fig.add_subplot(gs[3])
        ax_table.axis('off')

        # 1호기 (Control 고정)
        times1 = np.array(full_data[1]['time'])
        lens1 = np.array(full_data[1]['length'])
        cd1 = self.calc_cum_dist(lens1)
        th1 = np.array(full_data[1]['theta'])
        smc1 = np.array(full_data[1]['smc'])
        mask1 = self.get_mission_mask(times1, lens1)
        
        max_th1 = 0
        if np.any(mask1):
            cd1_p, th1_p, smc1_p = cd1[mask1], th1[mask1], smc1[mask1]
            max_th1 = np.max(th1_p)
            ax1.plot(cd1_p, th1_p, color='red', linewidth=0.8, label='Theta Mag (deg)')
            ax1.axhline(y=4.0, color='gray', linestyle=':', label='Threshold (4°)')
            ax1.set_title("Run: Control (Winch 1) - Theta vs Cumulative Distance", fontweight='bold', fontsize=13)
            ax1.set_ylabel("Theta Mag [deg]", fontsize=11)
            ax1.set_xlabel("Cumulative Distance [m]", fontsize=11)
            ax1.tick_params(axis='both', labelsize=10)
            ax1.grid(True, linestyle='--', alpha=0.6)
            ax1.legend(loc='upper right', fontsize=10)

            smc_binary1 = (smc1_p == 1).astype(int)
            ax2.step(cd1_p, smc_binary1, where='post', color='green', linewidth=0.8, label='Ctrl Active (1/0)')
            ax2.set_ylabel("Ctrl State", fontsize=11)
            ax2.set_xlabel("Cumulative Distance [m]", fontsize=11)
            ax2.set_ylim([-0.2, 1.2])
            ax2.set_yticks([0, 1])
            ax2.tick_params(axis='both', labelsize=10)
            ax2.grid(True, linestyle='--', alpha=0.6)
            ax2.legend(loc='upper right', fontsize=10)

        # 2호기 (Uncontrolled 고정)
        times2 = np.array(full_data[2]['time'])
        lens2 = np.array(full_data[2]['length'])
        cd2 = self.calc_cum_dist(lens2)
        th2 = np.array(full_data[2]['theta'])
        mask2 = self.get_mission_mask(times2, lens2)
        
        max_th2 = 0
        if np.any(mask2):
            cd2_p, th2_p = cd2[mask2], th2[mask2]
            max_th2 = np.max(th2_p)
            ax3.plot(cd2_p, th2_p, color='deepskyblue', linewidth=0.8, label='Theta Mag (deg)')
            ax3.axhline(y=4.0, color='gray', linestyle=':', label='Threshold (4°)')
            ax3.set_title("Run: Uncontrolled (Winch 2) - Theta vs Cumulative Distance", fontweight='bold', fontsize=13)
            ax3.set_xlabel("Cumulative Distance [m]", fontsize=11)
            ax3.set_ylabel("Theta Mag [deg]", fontsize=11)
            ax3.tick_params(axis='both', labelsize=10)
            ax3.grid(True, linestyle='--', alpha=0.6)
            ax3.legend(loc='upper right', fontsize=10)

        # Y축 통일
        max_y_lim = max(max_th1, max_th2) + 5
        if max_y_lim > 5:
            ax1.set_ylim(0, max_y_lim)
            ax3.set_ylim(0, max_y_lim)

        m1 = self.compute_metrics(full_data[1]) 
        m2 = self.compute_metrics(full_data[2]) 

        def calc_ratio(u_val, c_val):
            if u_val == 0: return "0.0% ↓"
            diff = ((u_val - c_val) / u_val) * 100.0
            symbol = "↓" if diff >= 0 else "↑"
            return f"{abs(diff):.1f}% {symbol}"

        table_data = [
            ["Mission Range", "RMSE (deg)", "Peak Angle\n(deg)", "Sway Energy\n(deg²·s)", "Ctrl Effort\n(m²/s³)", "Total Time\n(s)"],
            ["Control OFF", f"{m2[0]:.2f}", f"{m2[1]:.2f}", f"{m2[2]:.2f}", f"{m2[3]:.2e}", f"{m2[4]:.2f}"],
            ["SMC+tinyML", f"{m1[0]:.2f}", f"{m1[1]:.2f}", f"{m1[2]:.2f}", f"{m1[3]:.2e}", f"{m1[4]:.2f}"],
            ["Improvement", calc_ratio(m2[0], m1[0]), calc_ratio(m2[1], m1[1]), calc_ratio(m2[2], m1[2]), calc_ratio(m2[3], m1[3]), calc_ratio(m2[4], m1[4])]
        ]

        table = ax_table.table(cellText=table_data, loc='center', cellLoc='center')
        table.scale(1.0, 2.2)
        table.auto_set_font_size(False)
        table.set_fontsize(13)

        for (r, c), table_cell in table.get_celld().items():
            if r == 0:
                table_cell.set_facecolor('#808080')
                table_cell.get_text().set_color('white')
                table_cell.get_text().set_weight('bold')
            elif r == 1:
                table_cell.set_facecolor('#FFFFFF')
            elif r == 2:
                if c == 0:
                    table_cell.set_facecolor('#FFA500')
                    table_cell.get_text().set_weight('bold')
                else:
                    table_cell.set_facecolor('#FFFFFF')
            elif r == 3:
                table_cell.set_facecolor('#92D050')
                table_cell.get_text().set_weight('bold')
                table_cell.get_text().set_color('red')

        self.fig.tight_layout()
        self.canvas.draw()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = WinchGUI()
    window.show()
    sys.exit(app.exec_())