import serial
import time
import threading
import json
import queue
from pymavlink import mavutil
import paho.mqtt.client as mqtt
from datetime import datetime

ARDUCOPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
    5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT",
    14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW",
    19: "AVOID_ADSB", 20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD",
    23: "FOLLOW", 24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL"
}

MQTT_BROKER = 'test.mosquitto.org'
MQTT_PORT = 1883
MQTT_TOPIC = 'goodrich/drone/data'
PUBLISH_INTERVAL = 0.2  # seconds, set to desired publish rate (0.2 -> 5 Hz)
MAV_CONNECTION_STR ='udp:127.0.0.1:14550'  # 請依照實際調整  '/dev/ttyACM1'
USB_PORT = '/dev/ttyUSB0'             # 請依照實際調整        '/dev/ttyUSB0' 

# ---------------------------
# 電壓讀取 thread
# ---------------------------
class VoltageReader(threading.Thread):
    def __init__(self, port, baud=9600):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.latest_voltage = None
        self.running = True
        self.error = None
        self._last_ok_time = time.time()
        self._parse_err_count = 0
        self._ignore_list = ['START', 'INIT', 'OK', 'READY', '']

    def run(self):
        while self.running:
            try:
                with serial.Serial(self.port, self.baud, timeout=1) as ser:
                    self.error = None
                    print(f"[VoltageReader] 連線成功: {self.port}")
                    while self.running:
                        try:
                            line = ser.readline().decode('ascii', errors='ignore').strip()
                            if line:
                                try:
                                    voltage = float(line)
                                    self.latest_voltage = voltage
                                    self._last_ok_time = time.time()
                                    self._parse_err_count = 0
                                except ValueError:
                                    # 忽略常見雜訊或初始化訊息
                                    if line not in self._ignore_list:
                                        self._parse_err_count += 1
                                        if self._parse_err_count >= 3:
                                            self.error = f"Voltage parse err: {line}"
                                    else:
                                        self._parse_err_count = 0
                            time.sleep(0.05)
                        except Exception as e:
                            self.error = f"VoltageReader inner err: {e}"
            except Exception as e:
                self.error = f"Serial err: {e}"
                print(f"[VoltageReader] 失敗: {e}，3秒後重連...")
                time.sleep(3)

    def stop(self):
        self.running = False

    def is_timeout(self, sec=3):
        return time.time() - self._last_ok_time > sec

# ---------------------------
# MAVLink 讀取 thread
# ---------------------------
class MAVLinkReader(threading.Thread):
    def __init__(self, connection_str, msg_dict, lock):
        super().__init__(daemon=True)
        self.connection_str = connection_str
        self.msg_dict = msg_dict
        self.lock = lock
        self.running = True
        self.error = None
        self._last_ok_time = time.time()

    def run(self):
        while self.running:
            try:
                print("[MAVLinkReader] 嘗試連線...")
                master = mavutil.mavlink_connection(self.connection_str)
                master.wait_heartbeat(timeout=10)
                self.error = None
                print("[MAVLinkReader] 連線成功")
                # 設定訊息更新頻率
                for msgid in [
                    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
                    mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT
                ]:
                    master.mav.command_long_send(
                        master.target_system,
                        master.target_component,
                        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        0, msgid, 1000000, 0, 0, 0, 0, 0
                    )
                while self.running:
                    msg = master.recv_match(blocking=True, timeout=1)
                    if not msg or msg.get_type() == 'BAD_DATA':
                        continue
                    with self.lock:
                        self.msg_dict[msg.get_type()] = msg
                    self._last_ok_time = time.time()
            except Exception as e:
                self.error = f"MAVLink err: {e}"
                print(f"[MAVLinkReader] 失敗: {e}，3秒後重連...")
                time.sleep(3)

    def stop(self):
        self.running = False

    def is_timeout(self, sec=3):
        return time.time() - self._last_ok_time > sec

# ---------------------------
# 主控與 MQTT 發布
# ---------------------------
def main():
    # Thread-shared
    msg_dict = {}
    msg_lock = threading.Lock()

    voltage_reader = VoltageReader(USB_PORT)
    voltage_reader.start()

    mav_reader = MAVLinkReader(MAV_CONNECTION_STR, msg_dict, msg_lock)
    mav_reader.start()

    client = mqtt.Client()
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        print(f"[MQTT] 連線失敗: {e}")
        return
    # Start background network loop for paho-mqtt to handle reconnects/acks
    client.loop_start()
    print(f"[MQTT] 已連線 {MQTT_BROKER}:{MQTT_PORT} (background loop started)")

    # 抗抖動暫存
    last_mode = None
    mode_stable = None
    mode_stable_count = 0
    MODE_THRESHOLD = 2

    last_armed = None
    armed_stable = None
    armed_stable_count = 0
    ARMED_THRESHOLD = 2

    battery_present_last_time = 0

    try:
        while True:
            error_status = []
            # --- 檢查異常 ---
            if voltage_reader.error:
                error_status.append(voltage_reader.error)
            if voltage_reader.is_timeout():
                error_status.append("Voltage timeout/disconnected")
            if mav_reader.error:
                error_status.append(mav_reader.error)
            if mav_reader.is_timeout():
                error_status.append("MAVLink timeout/disconnected")

            # --- MAVLink資料 ---
            with msg_lock:
                messages = dict(msg_dict)  # 避免取值時與thread衝突

            now = time.time()
            if "SYS_STATUS" in messages:
                battery_present_last_time = now
            battery_present = (now - battery_present_last_time) < 10

            # ---- 組裝payload ----
            # 使用合理的預設值，避免前端接收 null 導致錯誤
            payload = {
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "flight_mode": None,
                "arm_status": None,
                "speed": 0.0,
                "altitude": 0.0,
                "battery": {
                    "volt": voltage_reader.latest_voltage,
                    "current": None,
                    "battery_percent": None,
                    "battery_present": battery_present
                },
                "gps_position": {
                    "lat": 0.0,
                    "lon": 0.0
                },
                "attitude": {
                    "pitch": 0.0,
                    "yaw": 0.0,
                    "roll": 0.0
                },
                "error_status": error_status if error_status else None
            }

            # 1. HEARTBEAT
            hb = messages.get("HEARTBEAT")
            if hb:
                custom_mode = getattr(hb, "custom_mode", 0)
                flight_mode = ARDUCOPTER_MODES.get(custom_mode, f"MODE_{custom_mode}")
                if flight_mode == last_mode:
                    mode_stable_count += 1
                else:
                    mode_stable_count = 0
                    last_mode = flight_mode
                if mode_stable_count >= MODE_THRESHOLD:
                    mode_stable = flight_mode

                is_armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                if is_armed == last_armed:
                    armed_stable_count += 1
                else:
                    armed_stable_count = 0
                    last_armed = is_armed
                if armed_stable_count >= ARMED_THRESHOLD:
                    armed_stable = is_armed

                payload["flight_mode"] = mode_stable if mode_stable else flight_mode
                payload["arm_status"] = "armed" if (armed_stable if armed_stable is not None else is_armed) else "disarmed"

            # 2. ATTITUDE
            att = messages.get("ATTITUDE")
            if att:
                payload["attitude"] = {
                    "pitch": round(att.pitch * 57.2958, 2),
                    "roll":  round(att.roll * 57.2958, 2),
                    "yaw":   round(att.yaw * 57.2958, 2)
                }

            # 3. GLOBAL_POSITION_INT
            gp = messages.get("GLOBAL_POSITION_INT")
            if gp:
                payload["gps_position"] = {
                    "lat": gp.lat / 1e7,
                    "lon": gp.lon / 1e7
                }
                payload["altitude"] = round(gp.relative_alt / 1000.0, 2)
            else:
                vh = messages.get("VFR_HUD")
                if vh:
                    payload["altitude"] = round(vh.alt, 2)
                    payload["speed"] = round(vh.groundspeed, 2)
            # 4. VFR_HUD (速度)
            vh = messages.get("VFR_HUD")
            if vh:
                payload["speed"] = round(vh.groundspeed, 2)

            # 5. SYS_STATUS
            bat = messages.get("SYS_STATUS")
            if bat:
                payload["battery"]["battery_percent"] = bat.battery_remaining
                payload["battery"]["current"] = (bat.current_battery / 100.0) if bat.current_battery != -1 else None
                payload["battery"]["battery_present"] = True
            else:
                # 若沒有 SYS_STATUS，可根據電壓回退判斷是否有電池讀值
                if voltage_reader.latest_voltage is not None:
                    payload["battery"]["battery_present"] = True
                else:
                    payload["battery"]["battery_present"] = False

            # 若沒有來自 SYS_STATUS 的 battery_percent，嘗試用電壓估算百分比
            if payload["battery"].get("battery_percent") is None and payload["battery"].get("volt"):
                try:
                    volt = payload["battery"]["volt"]
                    min_v, max_v = 14.8, 16.8
                    percent = max(0, min(100, (volt - min_v) / (max_v - min_v) * 100))
                    payload["battery"]["battery_percent"] = round(percent, 1)
                except Exception:
                    payload["battery"]["battery_percent"] = None

            # --- 發布MQTT ---
            try:
                client.publish(MQTT_TOPIC, json.dumps(payload))
                print(f"Published: {json.dumps(payload)}")
            except Exception as e:
                print(f"[MQTT] 發布失敗: {e}")
                error_status.append(f"MQTT publish error: {e}")

            # Control publish frequency via PUBLISH_INTERVAL
            time.sleep(PUBLISH_INTERVAL)

    except KeyboardInterrupt:
        print("已中斷執行")
    finally:
        voltage_reader.stop()
        mav_reader.stop()
        # Stop MQTT background loop then disconnect
        try:
            client.loop_stop()
        except Exception:
            pass
        client.disconnect()
        print("所有 thread 與資源已釋放")

if __name__ == '__main__':
    main()
