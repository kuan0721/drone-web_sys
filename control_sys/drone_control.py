import time
import sys
import signal
import json
import threading
from datetime import datetime
from pymavlink import mavutil
from control_sys.voltage_reader import VoltageReader

class LowBatteryResumeException(Exception):
    pass

class DroneController:
    def __init__(self, connection_string, voltage_port, voltage_threshold=15.2,
                voltage_baud=9600, takeoff_altitude=15, square_size=10):
        self.connection_string = connection_string
        self.voltage_threshold = voltage_threshold
        self.takeoff_altitude = takeoff_altitude
        self.square_size = square_size

        # MAVLink
        self.master = None
        self.initial_yaw_pre_takeoff = None
        self.recorded_position = None
        self.recorded_next_waypoint = None
        self.recorded_yaw_low_battery = None
        self.next_waypoint = None

        # Voltage reader
        self.voltage_reader = VoltageReader(port=voltage_port, baud=voltage_baud)
        self.voltage_reader.register_callback(threshold=self.voltage_threshold,
                                            callback=self._on_low_voltage)

        # 充電歷程記錄
        self.charging_data = []
        self.charging_start_time = None
        self.is_charging = False
        self.charging_thread = None
        self.charging_stop_flag = threading.Event()

    def connect(self):
        self.master = mavutil.mavlink_connection(self.connection_string)
        self.master.wait_heartbeat()
        print("Connected to drone via MAVLink!")
        self.voltage_reader.start()
        print(f"Started voltage reader on {self.voltage_reader.port}")

    def _on_low_voltage(self, voltage):
        print(f"⚠️ Voltage {voltage:.3f} V below threshold {self.voltage_threshold} V, initiating low-battery RTL")
        self.recorded_position = self.get_gps_position()
        self.recorded_next_waypoint = self.next_waypoint
        self.recorded_yaw_low_battery = self.get_initial_yaw()
        self.low_battery_rtl()
        raise LowBatteryResumeException()

    def start_charging_monitor(self):
        if self.is_charging:
            return
        self.is_charging = True
        self.charging_start_time = datetime.now()
        self.charging_stop_flag.clear()
        self.charging_data = []
        print("🔋 開始充電監控...")
        self.charging_thread = threading.Thread(target=self._charging_monitor_loop, daemon=True)
        self.charging_thread.start()

    def stop_charging_monitor(self):
        if not self.is_charging:
            return
        self.is_charging = False
        self.charging_stop_flag.set()
        self.save_charging_history()
        print("🔋 充電監控已停止")

    def _charging_monitor_loop(self):
        try:
            while not self.charging_stop_flag.is_set():
                current_voltage = self.voltage_reader.latest_voltage if self.voltage_reader.latest_voltage else 0
                elapsed_time = (datetime.now() - self.charging_start_time).total_seconds()
                voltage_range = 16.8 - 15.2
                voltage_above_min = max(0, current_voltage - 15.2)
                battery_percent = min(100, (voltage_above_min / voltage_range) * 100)
                charging_point = {
                    "timestamp": datetime.now().isoformat(),
                    "elapsed_seconds": elapsed_time,
                    "voltage": current_voltage,
                    "battery_percent": round(battery_percent, 1),
                    "charging_rate": self._calculate_charging_rate()
                }
                self.charging_data.append(charging_point)
                if len(self.charging_data) % 5 == 0:
                    self.save_charging_history()
                if current_voltage >= 16.5:
                    print(f"🔋 充電完成！最終電壓: {current_voltage:.2f}V")
                    break
                time.sleep(2)
        except Exception as e:
            print(f"充電監控錯誤: {e}")
        finally:
            self.stop_charging_monitor()

    def _calculate_charging_rate(self):
        if len(self.charging_data) < 2:
            return 0
        current = self.charging_data[-1]
        previous = self.charging_data[-2]
        voltage_diff = current["voltage"] - previous["voltage"]
        time_diff = current["elapsed_seconds"] - previous["elapsed_seconds"]
        if time_diff > 0:
            return (voltage_diff / time_diff) * 60
        return 0

    def save_charging_history(self):
        charging_history = {
            "session_id": self.charging_start_time.strftime("%Y%m%d_%H%M%S"),
            "start_time": self.charging_start_time.isoformat(),
            "end_time": datetime.now().isoformat() if not self.is_charging else None,
            "total_duration_seconds": (datetime.now() - self.charging_start_time).total_seconds(),
            "data_points": self.charging_data,
            "summary": {
                "initial_voltage": self.charging_data[0]["voltage"] if self.charging_data else 0,
                "final_voltage": self.charging_data[-1]["voltage"] if self.charging_data else 0,
                "initial_percent": self.charging_data[0]["battery_percent"] if self.charging_data else 0,
                "final_percent": self.charging_data[-1]["battery_percent"] if self.charging_data else 0,
                "avg_charging_rate": sum(point["charging_rate"] for point in self.charging_data) / len(self.charging_data) if self.charging_data else 0
            }
        }
        try:
            with open('charging_history.json', 'w', encoding='utf-8') as f:
                json.dump(charging_history, f, indent=2, ensure_ascii=False)
            print(f"充電記錄已保存 ({len(self.charging_data)} 個資料點)")
        except Exception as e:
            print(f"保存充電記錄失敗: {e}")

    # ========== 正方形巡航任務 ==========
    def fly_square(self):
        print("開始繞正方形巡航")
        altitude = self.takeoff_altitude
        size = self.square_size

        # 依 LOCAL_NED 框架，(x, y, z) 單位為公尺
        waypoints = [
            (0, 0, altitude),
            (size, 0, altitude),
            (size, size, altitude),
            (0, size, altitude),
            (0, 0, altitude)
        ]
        for i, point in enumerate(waypoints):
            print(f"  ➤ 前往第{i+1}點 {point}")
            self.fly_to_point(*point)

    # ================= 其餘原有方法保持不變 =================
    def get_arm_status(self):
        hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            return "armed" if armed else "disarmed"
        return "unknown"

    def get_initial_yaw(self):
        msg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=2)
        return msg.heading if msg else 0

    def get_gps_position(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        return (msg.lat / 1e7, msg.lon / 1e7) if msg else (0, 0)

    def rotate_yaw(self, angle, relative=0):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0, angle,10, relative,0,0,0,0
        )
        time.sleep(8)

    def arm_and_takeoff(self):
        self.initial_yaw_pre_takeoff = self.get_initial_yaw()
        print(f"Recorded pre-takeoff yaw: {self.initial_yaw_pre_takeoff}")

        self.master.set_mode_apm('GUIDED')
        time.sleep(1)
        for _ in range(5):
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1,0,0,0,0,0,0
            )
            time.sleep(2)
            if self.get_arm_status() == "armed":
                print("Armed")
                break
        else:
            print("Arming failed.")
            sys.exit(1)

        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0,0,0,0,0,0, self.takeoff_altitude
        )
        print(f"Taking off to {self.takeoff_altitude}m...")
        time.sleep(10)

    def fly_to_point(self, x, y, z):
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000), x, y, -z, 0,0,0,0,0,0,0,0
        )
        print(f"Flying to {x},{y},{z}")
        time.sleep(10)

    def _rtl_and_land(self):
        print("開始 RTL → GUIDED → 轉航向 yaw → RTL 降落流程")
        self.master.set_mode_apm('RTL')
        print("已切換至 RTL 模式，等待確認")
        while True:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True)
            if hb and hb.custom_mode == 6:
                print("確認進入 RTL 模式")
                break
            time.sleep(0.5)

        adjusted = False
        landed = False
        while not landed:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if not msg:
                continue
            alt = msg.relative_alt / 1000.0
            if alt <= 9 and not adjusted:
                self.master.set_mode_apm('GUIDED')
                print("切換至 GUIDED 模式")
                time.sleep(1)
                self.rotate_yaw(self.initial_yaw_pre_takeoff)
                print(f"完成航向轉向 yaw={self.initial_yaw_pre_takeoff}")
                self.master.set_mode_apm('RTL')
                print("切回 RTL 模式，繼續自動降落")
                adjusted = True
            if adjusted and alt <= 0.2:
                print("降落完成")
                landed = True
            time.sleep(0.2)

    def low_battery_rtl(self):
        print("Low-Battery RTL...")
        self._rtl_and_land()
        self.wait_for_charging_complete()
        self.resume_mission()

    def wait_for_charging_complete(self):
        print("⏳ 等待充電完成...")
        while self.is_charging:
            time.sleep(5)
            if self.charging_data:
                latest = self.charging_data[-1]
                print(f"充電進度: {latest['battery_percent']:.1f}% ({latest['voltage']:.2f}V)")
        print("✅ 充電完成，準備恢復任務")
        time.sleep(2)

    def emergency_rtl(self):
        print("Emergency RTL: immediate landing")
        self._rtl_and_land()
        print("Emergency landing complete.")
        sys.exit(0)

    def final_rtl(self):
        print("Final RTL: landing procedure...")
        self._rtl_and_land()
        print("Final landing complete.")
        sys.exit(0)

    def return_to_launch(self):
        self.emergency_rtl()

    def resume_mission(self):
        print("Resuming mission...")
        self.stop_charging_monitor()
        self.arm_and_takeoff()
        lat, lon = self.recorded_position
        self.master.mav.set_position_target_global_int_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b0000111111111000), int(lat*1e7), int(lon*1e7), self.takeoff_altitude,
            0,0,0,0,0,0,0,0
        )
        time.sleep(15)
        self.rotate_yaw(self.recorded_yaw_low_battery)
        print("Mission resumed.")

# ==================== 主控流程範例 ====================

if __name__ == "__main__":
    # 請根據實際參數調整
    drone = DroneController(
        connection_string='/dev/ttyACM1',
        voltage_port='/dev/ttyUSB0',
        voltage_threshold=15.2,
        voltage_baud=9600,
        takeoff_altitude=15,
        square_size=10       # <--- 設定正方形邊長（公尺）
    )

    drone.connect()
    drone.arm_and_takeoff()
    drone.fly_square()       # <--- 執行正方形自動巡航

    # ...任務結束可自動 RTL
    drone.final_rtl()
