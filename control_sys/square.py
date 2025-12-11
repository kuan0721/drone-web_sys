import time
import sys
import signal
import math
from pymavlink import mavutil

class DroneController:
    def __init__(self, connection_string, takeoff_altitude=15, square_size=10):
        self.connection_string = connection_string
        self.takeoff_altitude = takeoff_altitude
        self.square_size = square_size

        # MAVLink 連線物件
        self.master = None
        
        # 狀態變數
        self.initial_yaw_pre_takeoff = 0
        self.is_running = True  # 控制主迴圈旗標

    # ================= 連線與基礎功能 =================
    def connect(self):
        print(f"Connecting to {self.connection_string}...")
        self.master = mavutil.mavlink_connection(self.connection_string)
        self.master.wait_heartbeat()
        print(">> Connected to drone via MAVLink!")

    def get_local_position(self):
        """ 獲取當前 NED 相對座標 (x, y, z) """
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg:
            return (msg.x, msg.y, msg.z)
        return None

    def get_heading(self):
        """ 讀取當前機頭朝向 (0-360) """
        msg = self.master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
        if msg:
            return msg.heading
        return None

    def get_arm_status(self):
        hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            return "armed" if armed else "disarmed"
        return "unknown"

    # ================= 閉迴圈檢查工具 (關鍵修正) =================
    def wait_reached_target(self, target_x, target_y, target_z, timeout=30):
        """
        改良版等待函式：
        1. 分離 XY (水平) 與 Z (高度) 的判定，避免高度飄移導致卡關。
        2. 加入 timeout 機制，避免永久卡在某一點。
        """
        start_time = time.time()
        print(f">> 導航監控啟動... (超時設定: {timeout}s)")
        
        while self.is_running:
            pos = self.get_local_position()
            if not pos:
                time.sleep(0.1)
                continue

            curr_x, curr_y, curr_z = pos
            
            # 計算誤差
            dist_xy = math.sqrt((target_x - curr_x)**2 + (target_y - curr_y)**2)
            dist_z = abs(target_z - curr_z)

            # 顯示即時數據 (每 0.5 秒刷新一次)
            if (time.time() - start_time) % 0.5 < 0.1:
                sys.stdout.write(f"\r   [飛行中] 水平誤差: {dist_xy:.1f}m | 高度誤差: {dist_z:.1f}m | 剩餘時間: {int(timeout - (time.time() - start_time))}s   ")
                sys.stdout.flush()

            # 判定標準：水平 < 1.5m 且 高度 < 3.0m (高度誤差容忍度較大)
            if dist_xy < 1.5 and dist_z < 3.0:
                print(f"\n>> 抵達目標點 (XY誤差:{dist_xy:.2f}m)")
                break
            
            # 超時強制跳過
            if time.time() - start_time > timeout:
                print(f"\n>> [警告] 導航超時！強制視為到達，前往下一動作。")
                break
            
            time.sleep(0.1)

    def wait_yaw_aligned(self, target_yaw, threshold=5, timeout=20):
        """
        等待機頭轉向至目標角度 (解決降落轉向不準問題)
        """
        print(f">> 等待航向對齊至 {target_yaw}° (誤差容許 {threshold}°)...")
        start_time = time.time()

        while self.is_running:
            current_yaw = self.get_heading()
            if current_yaw is None:
                time.sleep(0.2)
                continue

            # 計算 0-360 度環形的最短角度差
            diff = abs(target_yaw - current_yaw)
            if diff > 180:
                diff = 360 - diff
            
            if (time.time() - start_time) % 0.5 < 0.1:
                sys.stdout.write(f"\r   [轉向中] 目標:{target_yaw} 當前:{current_yaw} 誤差:{diff}°   ")
                sys.stdout.flush()

            if diff <= threshold:
                print(f"\n>> 航向已對齊！")
                return True

            if time.time() - start_time > timeout:
                print(f"\n>> [警告] 轉向超時！繼續執行降落。")
                return False

            time.sleep(0.2)

    # ================= 動作指令 =================
    def rotate_yaw(self, angle, relative=0):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0, angle, 0, relative, 0, 0, 0, 0 # speed設0由飛控決定
        )

    def arm_and_takeoff(self):
        # 1. 記錄起飛前的 Yaw
        self.initial_yaw_pre_takeoff = self.get_heading()
        print(f"Recorded pre-takeoff yaw: {self.initial_yaw_pre_takeoff}")

        self.master.set_mode_apm('GUIDED')
        time.sleep(1)

        # 2. 解鎖
        print("Arming drone...")
        for _ in range(5):
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            time.sleep(1)
            if self.get_arm_status() == "armed":
                print(">> Drone Armed!")
                break
        else:
            print("Arming failed. Exiting.")
            sys.exit(1)

        # 3. 起飛
        print(f"Taking off to {self.takeoff_altitude}m...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, self.takeoff_altitude
        )
        
        # 等待起飛完成 (直接使用 target height 作為 Z)
        # 注意：wait_reached_target 的 z 是負數 (NED)
        self.wait_reached_target(0, 0, -self.takeoff_altitude, timeout=20)

    def fly_to_point(self, x, y, z):
        print(f"\nCmd: Flying to NED ({x}, {y}, {z})")
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000), 
            x, y, -z, # NED Z軸向下為正，所以輸入高度要轉負
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.wait_reached_target(x, y, -z)

    def fly_square(self):
        print("\n=== 開始繞正方形巡航 ===")
        alt = self.takeoff_altitude
        size = self.square_size

        # 定義四個點 (相對起飛點)
        waypoints = [
            (size, 0, alt),      # 北
            (size, size, alt),   # 東
            (0, size, alt),      # 南 (回 X=0)
            (0, 0, alt)          # 西 (回原點)
        ]
        
        for i, point in enumerate(waypoints):
            print(f"➤ Waypoint {i+1}: {point}")
            self.fly_to_point(*point)
            print("   懸停 1 秒...")
            time.sleep(1) 

    # ================= 降落與緊急處理 =================
    def _rtl_and_land(self):
        print("\n=== 開始智慧降落流程 (Smart RTL) ===")
        self.master.set_mode_apm('RTL')
        
        # 確認切換成功
        while True:
            hb = self.master.recv_match(type='HEARTBEAT', blocking=True)
            if hb and hb.custom_mode == 6: # RTL mode ID
                print(">> 確認進入 RTL 模式，返航中...")
                break
            time.sleep(0.5)

        adjusted = False
        landed = False
        
        while not landed and self.is_running:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if not msg: continue
            
            alt = msg.relative_alt / 1000.0 # mm -> m

            # --- 轉向邏輯 ---
            if alt <= 9 and not adjusted:
                print(f"\n>> 高度 {alt:.1f}m，執行機頭轉正...")
                
                # 1. 切換 GUIDED 以便控制 Yaw
                self.master.set_mode_apm('GUIDED')
                time.sleep(1)
                
                # 2. 發送指令
                print(f">> 轉向至 {self.initial_yaw_pre_takeoff}°")
                self.rotate_yaw(self.initial_yaw_pre_takeoff)
                
                # 3. 等待確實轉到 (閉迴圈)
                self.wait_yaw_aligned(self.initial_yaw_pre_takeoff, threshold=5)
                
                # 4. 切回 RTL
                print(">> 轉向完成，切回 RTL 繼續降落")
                self.master.set_mode_apm('RTL')
                adjusted = True
            
            # --- 落地判定 ---
            if alt <= 0.3:
                print(f"\n>> 偵測到地面 (Alt: {alt:.2f}m)，等待上鎖...")
                landed = True 
            
            time.sleep(0.2)
        print("降落程序完成。")

    def emergency_rtl(self):
        """ 當程式被中斷時呼叫 """
        print("\n!!! 緊急中斷 (Emergency RTL) !!!")
        self.is_running = False
        self.master.set_mode_apm('RTL')
        sys.exit(0)

# ==================== 主程式 ====================
drone = None

def signal_handler(sig, frame):
    if drone:
        drone.emergency_rtl()

if __name__ == "__main__":
    # 註冊 Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # 設定參數 (模擬器預設 Port)
    # 如果是實機，請改為 '/dev/ttyACM0' 或 '/dev/serial0' 等
    connection_str = 'udp:127.0.0.1:14550' 
    
    drone = DroneController(
        connection_string=connection_str, 
        takeoff_altitude=15, # 起飛高度
        square_size=10       # 正方形邊長
    )

    try:
        drone.connect()
        drone.arm_and_takeoff()
        drone.fly_square()
        drone._rtl_and_land()
    except Exception as e:
        print(f"\nError: {e}")
        drone.emergency_rtl()