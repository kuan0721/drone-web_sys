import time
import sys
import signal
from pymavlink import mavutil

class DroneController:
    def __init__(self, connection_string, takeoff_altitude=15, square_size=10):
        self.connection_string = connection_string
        self.takeoff_altitude = takeoff_altitude
        self.square_size = square_size

        # MAVLink
        self.master = None
        self.initial_yaw_pre_takeoff = None
        self.recorded_position = None
        self.recorded_next_waypoint = None
        self.recorded_yaw_low_battery = None
        self.next_waypoint = None

    def connect(self):
        self.master = mavutil.mavlink_connection(self.connection_string)
        self.master.wait_heartbeat()
        print("Connected to drone via MAVLink!")

    # ========== 新增正方形巡航任務 ==========
    def fly_square(self):
        print("開始繞正方形巡航")
        altitude = self.takeoff_altitude
        size = self.square_size

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

    # ================= 其餘控制 =================
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
            0, angle, 10, relative, 0, 0, 0, 0
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
                0, 1, 0, 0, 0, 0, 0, 0
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
            0, 0, 0, 0, 0, 0, 0, self.takeoff_altitude
        )
        print(f"Taking off to {self.takeoff_altitude}m...")
        time.sleep(10)

    def fly_to_point(self, x, y, z):
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000), x, y, -z, 0, 0, 0, 0, 0, 0, 0, 0
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

# ==================== 主控流程範例 ====================

if __name__ == "__main__":
    # 請根據實際參數調整
    drone = DroneController(
        connection_string='/dev/ttyACM1',   # ← 模擬器通常 /dev/ttyACM0 或 udp:127.0.0.1:14550
        takeoff_altitude=15,
        square_size=5       # <--- 設定正方形邊長（公尺）
    )

    drone.connect()
    drone.arm_and_takeoff()
    drone.fly_square()       # <--- 執行正方形自動巡航
    drone.final_rtl()        # <--- 自動 RTL 降落