import time
import sys
import signal
import json
import threading
from datetime import datetime

from pymavlink import mavutil
from voltage_reader import VoltageReader


class LowBatteryResumeException(Exception):
    pass


class DroneController:
    def __init__(self, connection_string, voltage_port, voltage_threshold=15.2,
                 voltage_baud=9600, takeoff_altitude=15, square_size=10):
        self.connection_string = connection_string
        self.voltage_threshold = voltage_threshold
        self.takeoff_altitude = takeoff_altitude
        self.square_size = square_size

        # MAVLink 相關
        self.master = None
        self.initial_yaw_pre_takeoff = None
        self.recorded_position = None
        self.recorded_next_waypoint = None
        self.recorded_yaw_low_battery = None
        self.next_waypoint = None

        # 電壓監控
        self.voltage_reader = VoltageReader(port=voltage_port, baud=voltage_baud)
        self.voltage_reader.register_callback(
            threshold=self.voltage_threshold,
            callback=self._on_low_voltage
        )

        # 充電歷程記錄
        self.charging_data = []
        self.charging_start_time = None
        self.is_charging = False
        self.charging_thread = None
        self.charging_stop_flag = threading.Event()

    # ================= 連線與電壓監控 =================

    def connect(self):
        self.master = mavutil.mavlink_connection(self.connection_string)
        self.master.wait_heartbeat()
        print("Connected to drone via MAVLink!")

        # 額外讀一次 HEARTBEAT 以檢視 autopilot 型別與模式
        hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb:
            try:
                mode_str = mavutil.mode_string_apm(hb)
            except Exception:
                mode_str = f"UNKNOWN(custom_mode={hb.custom_mode})"
            print(
                f"Heartbeat: type={hb.type}, autopilot={hb.autopilot}, "
                f"base_mode={hb.base_mode}, custom_mode={hb.custom_mode}, mode={mode_str}"
            )
        else:
            print("⚠️ 無法取得額外 HEARTBEAT，模式偵測資訊有限")

        self.voltage_reader.start()
        print(f"Started voltage reader on {self.voltage_reader.port}")

    def _on_low_voltage(self, voltage):
        print(f"⚠️ Voltage {voltage:.3f} V below threshold {self.voltage_threshold} V, initiating low-battery RTL")
        self.recorded_position = self.get_gps_position()
        self.recorded_next_waypoint = self.next_waypoint
        self.recorded_yaw_low_battery = self.get_initial_yaw()
        self.low_battery_rtl()
        raise LowBatteryResumeException()

    # ================= 共用：以「模式名稱字串」切換 / 確認模式 =================

    def set_mode_str(self, mode_name, timeout=5.0):
        """
        使用 set_mode_apm(字串) 切換模式，並透過 HEARTBEAT + mode_string_apm
        以模式名稱確認是否切換成功。
        這樣就不再假設 custom_mode 的數值（例如 RTL=6、GUIDED=4），
        對 ArduPilot 系列相對穩定。
        """
        print(f"嘗試切換飛行模式為 {mode_name}")

        # 1) 下指令
        try:
            self.master.set_mode_apm(mode_name)
        except Exception as e:
            print(f"切換模式 {mode_name} 時發生錯誤: {e}")
            return False

        # 2) 透過 HEARTBEAT 確認模式
        t0 = time.time()
        last_print = 0
        while time.time() - t0 < timeout:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if not msg:
                continue

            try:
                current_mode = mavutil.mode_string_apm(msg)
            except Exception:
                current_mode = f"UNKNOWN(custom_mode={msg.custom_mode})"

            now = time.time()
            if now - last_print > 1.0:
                print(
                    f"  當前 HEARTBEAT 模式: {current_mode}, "
                    f"base_mode={msg.base_mode}, custom_mode={msg.custom_mode}"
                )
                last_print = now

            if current_mode == mode_name:
                print(f"✔ 已確認切換為 {mode_name}")
                return True

        print(f"⚠️ 未能在 {timeout:.1f} 秒內確認切換為 {mode_name}")
        print("   → 請在地面站檢查實際 flight mode 是否有變化，以及確認為 ArduPilot 系列")
        return False

    # ================= 充電監控 =================

    def start_charging_monitor(self):
        if self.is_charging:
            return
        self.is_charging = True
        self.charging_start_time = datetime.now()
        self.charging_stop_flag.clear()
        self.charging_data = []
        print("🔋 開始充電監控...")
        self.charging_thread = threading.Thread(
            target=self._charging_monitor_loop,
            daemon=True
        )
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

    # ================= 正方形巡航 =================

    def fly_square(self):
        print("開始繞正方形巡航（以當前位置為原點）")
        size = self.square_size

        # LOCAL_OFFSET_NED，相對位移四個邊
        waypoints_offset = [
            ( size,  0, 0),   # 往前 size 公尺
            ( 0,   size, 0),  # 往右 size 公尺
            (-size, 0,   0),  # 往後 size 公尺
            ( 0,  -size, 0)   # 往左 size 公尺，回到起點上空
        ]

        for i, (dx, dy, dz) in enumerate(waypoints_offset):
            print(f"  ➤ 第 {i+1} 段，相對位移 (dx={dx}, dy={dy}, dz={dz})")
            self.fly_offset(dx, dy, dz)

    def fly_offset(self, dx, dy, dz):
        """
        使用 LOCAL_OFFSET_NED，相對當前位置做位移。
        dx, dy, dz 單位為公尺；z 向下為正，因此保持高度時 dz=0。
        """
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            int(0b110111111000),
            dx, dy, dz,
            0, 0, 0,
            0, 0, 0, 0, 0
        )
        print(f"Flying offset: dx={dx}, dy={dy}, dz={dz}")
        time.sleep(10)

    def fly_to_point(self, x, y, z):
        """
        若需要使用絕對 LOCAL_NED 座標，可使用此版本。
        """
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000),
            x, y, -z,
            0, 0, 0,
            0, 0, 0, 0, 0
        )
        print(f"Flying to {x},{y},{z}")
        time.sleep(10)

    # ================= 狀態讀取 =================

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
        """
        angle: 目標 yaw（絕對角度 0~360，或相對角度）
        relative: 0=絕對, 1=相對
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            angle,
            10,
            relative,
            0, 0, 0, 0
        )
        time.sleep(8)

    # ================= 起飛 =================

    def arm_and_takeoff(self):
        self.initial_yaw_pre_takeoff = self.get_initial_yaw()
        print(f"Recorded pre-takeoff yaw: {self.initial_yaw_pre_takeoff}")

        # 切 GUIDED
        self.set_mode_str("GUIDED")

        for _ in range(5):
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            time.sleep(2)
            if self.get_arm_status() == "armed":
                print("Armed")
                break
        else:
            print("Arming failed.")
            sys.exit(1)

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            self.takeoff_altitude
        )
        print(f"Taking off to {self.takeoff_altitude}m...")
        time.sleep(10)

    # ================= RTL 與降落 =================

    def _rtl_and_land(self):
        """
        RTL 降落流程：
        1. 切 RTL 並確認進入。
        2. 監控高度，當下降到門檻（預設 9m）：
           - 切 GUIDED
           - 在 GUIDED 模式下用 CONDITION_YAW 轉回起飛前 yaw
           - 再切回 RTL，讓飛控自動降落。
        """
        print("開始 RTL → GUIDED → 轉航向 yaw → RTL 降落流程")

        # 第一步：切換至 RTL
        self.set_mode_str("RTL")

        adjusted = False
        landed = False

        while not landed:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if not msg:
                continue

            alt = msg.relative_alt / 1000.0  # 相對高度（公尺）

            # 高度達門檻：GUIDED 調整 yaw，再回 RTL
            if alt <= 9 and not adjusted:
                print(f"高度約 {alt:.1f} m → 切換 GUIDED 以調整航向")
                if not self.set_mode_str("GUIDED"):
                    print("⚠️ 無法切換 GUIDED，放棄調整 yaw，維持 RTL 降落")
                else:
                    if self.initial_yaw_pre_takeoff is not None:
                        self.rotate_yaw(self.initial_yaw_pre_takeoff, relative=0)
                        print(f"已在 GUIDED 模式下轉回起飛前 yaw={self.initial_yaw_pre_takeoff}")
                    else:
                        print("⚠️ initial_yaw_pre_takeoff 為 None，略過航向調整")

                    # 轉向完成後再切回 RTL
                    self.set_mode_str("RTL")
                    print("轉向完成，切回 RTL 繼續自動降落")

                adjusted = True

            # 判斷已接近著陸
            if alt <= 0.2:
                print("降落完成（RTL）")
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
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            int(0b0000111111111000),
            int(lat * 1e7),
            int(lon * 1e7),
            self.takeoff_altitude,
            0, 0, 0,
            0, 0, 0, 0, 0
        )
        time.sleep(15)
        self.rotate_yaw(self.recorded_yaw_low_battery)
        print("Mission resumed.")


# ==================== 訊號處理：程式被中斷時自動 RTL ====================

def _register_signal_handlers(drone: DroneController):
    """
    當程式被中斷 (SIGINT / SIGTERM) 時，
    立刻啟動 emergency_rtl()，走 RTL 降落流程。
    """

    def handler(sig, frame):
        print(f"\n⚠️ 收到系統中斷訊號 ({sig})，啟動緊急 RTL 降落程序...")
        try:
            drone.emergency_rtl()
        except SystemExit:
            raise
        except Exception as e:
            print(f"緊急 RTL 過程發生錯誤: {e}")
            sys.exit(1)

    signal.signal(signal.SIGINT, handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, handler)  # 外部 kill


# ==================== 主控流程 ====================

if __name__ == "__main__":
    # 請根據實際參數調整
    drone = DroneController(
        connection_string='udp:127.0.0.1:14550',  # 實機可能為 /dev/ttyACM0
        voltage_port='/dev/ttyUSB0',
        voltage_threshold=15.2,
        voltage_baud=9600,
        takeoff_altitude=15,
        square_size=10        # 正方形邊長（公尺）
    )

    _register_signal_handlers(drone)

    try:
        drone.connect()
        drone.arm_and_takeoff()
        drone.fly_square()
        drone.final_rtl()

    except LowBatteryResumeException:
        print("低電壓 RTL 流程啟動，後續由程式內部管理。")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt 捕捉到，已嘗試啟動緊急 RTL 降落。")
        drone.emergency_rtl()

    except Exception as e:
        print(f"主流程發生未預期錯誤: {e}，啟動緊急 RTL 降落。")
        drone.emergency_rtl()
