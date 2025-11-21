# app.py
import os
import json
import subprocess
import sys
import time
import threading

from flask import Flask, jsonify, render_template, request
import paho.mqtt.client as mqtt

# ─── 啟動 web_data.py 子程序 ───
socket_server_process = subprocess.Popen([sys.executable, "web_data.py"])
time.sleep(1)

app = Flask(__name__)

# ─── 資料檔案路徑 ───
DATA_FILE      = 'drone_data.json'
CHARGING_FILE  = 'charging_history.json'

# 若 drone_data.json 不存在，就建立空檔
if not os.path.exists(DATA_FILE):
    with open(DATA_FILE, 'w', encoding='utf-8') as f:
        json.dump({}, f)

# ─── 輔助：讀取最新 drone_data.json ───
def load_data():
    with open(DATA_FILE, 'r', encoding='utf-8') as f:
        try:
            return json.load(f)
        except json.JSONDecodeError:
            return {}

# ─── 輔助：讀取充電歷程 ───
def load_charging_data():
    try:
        with open(CHARGING_FILE, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return []

# ─── 全域 MQTT 狀態變數 ───
last_status = ""

MQTT_BROKER = "test.mosquitto.org"
MQTT_PORT   = 1883
MQTT_TOPIC  = "goodrich/drone/status"

def on_connect(client, userdata, flags, rc):
    print(f"🔗 已連線到 MQTT Broker (rc={rc})，開始訂閱：{MQTT_TOPIC}")
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    global last_status
    last_status = msg.payload.decode("utf-8")
    print(f"[MQTT] 收到狀態：{last_status}")

def start_mqtt_client():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_forever()

# 啟動 MQTT 監聽執行緒（Daemon）
threading.Thread(target=start_mqtt_client, daemon=True).start()

# ─────────────────────────────────────────────
#                 Flask 路由
# ─────────────────────────────────────────────

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/get_flight_mode")
def get_flight_mode():
    data = load_data()
    return jsonify({"flight_mode": data.get("flight_mode", "unknown")})

@app.route("/get_arming_status")
def get_arming_status():
    data = load_data()
    return jsonify({"arm_status": data.get("arm_status", "unknown")})

@app.route("/get_speed")
def get_speed():
    data = load_data()
    return jsonify({"speed": data.get("speed", 0.0)})

@app.route("/get_altitude")
def get_altitude():
    data = load_data()
    return jsonify({"altitude": data.get("altitude", 0.0)})

@app.route("/get_battery")
def get_battery():
    data = load_data()
    batt = data.get("battery", {})
    volt = batt.get("volt", 0.0)
    # 計算 battery_percent
    min_v, max_v = 14.8, 16.8
    percent = 0.0
    if volt:
        percent = max(0, min(100, (volt - min_v) / (max_v - min_v) * 100))
        percent = round(percent, 1)
    return jsonify({
        "volt": volt,
        "current": batt.get("current", 0.0),
        "battery_percent": percent,
        "battery_present": batt.get("battery_present", False)
    })

@app.route("/get_rtk_status")
def get_rtk_status():
    data = load_data()
    return jsonify({"rtk_status": data.get("rtk_status", "unknown")})

@app.route("/get_drone_position")
def get_drone_position():
    data = load_data()
    gps = data.get("gps_position", {})
    return jsonify({
        "lat": gps.get("lat", 0.0),
        "lon": gps.get("lon", 0.0)
    })

@app.route("/get_drone_orientation")
def get_drone_orientation():
    data = load_data()
    att = data.get("attitude", {})
    return jsonify({
        "pitch": att.get("pitch", 0.0),
        "yaw": att.get("yaw", 0.0),
        "roll": att.get("roll", 0.0)
    })

@app.route("/get_charge_data")
def get_charge_data():
    data = load_data()
    batt = data.get("battery", {})
    voltage = batt.get("volt", 0.0)
    # 假設 14V=0%、16.8V=100%
    charge_percent = 0
    if voltage:
        charge_percent = max(0, min(100, (voltage - 14.0) / (16.8 - 14.0) * 100))
        charge_percent = round(charge_percent, 1)
    return jsonify({
        "voltage": voltage,
        "current": batt.get("current", 0.0),
        "battery_percent": charge_percent
    })

@app.route("/get_charging_history")
def get_charging_history():
    return jsonify({
        "charging_sessions": load_charging_data(),
        "total_sessions": len(load_charging_data())
    })

@app.route("/get_latest_charging_curve")
def get_latest_charging_curve():
    charging_data = load_charging_data()
    if not charging_data:
        return jsonify({"labels": [], "data": [], "session_info": None})

    latest = charging_data[-1]
    curve = latest.get('charging_curve', [])
    labels = []
    data   = []
    for pt in curve:
        # time_minutes to MM:SS
        m = int(pt['time_minutes'])
        s = int((pt['time_minutes'] - m) * 60)
        labels.append(f"{m}:{s:02d}")
        data.append(pt['percentage'])
    session_info = {
        'start_time':       latest.get('start_time'),
        'duration_minutes': latest.get('duration_minutes', 0),
        'start_voltage':    latest.get('start_voltage', 0),
        'end_voltage':      latest.get('end_voltage', 0),
        'is_complete':      latest.get('end_time') is not None
    }
    return jsonify({"labels": labels, "data": data, "session_info": session_info})

@app.route("/save_charging_record", methods=["POST"])
def save_charging_record():
    record = request.json
    history = load_charging_data()
    history.append(record)
    with open(CHARGING_FILE, 'w', encoding='utf-8') as f:
        json.dump(history, f, ensure_ascii=False, indent=2)
    return jsonify({"status": "ok"})

@app.route("/get_charge_status")
def get_charge_status():
    """前端輪詢最新 MQTT 狀態"""
    return jsonify({"status": last_status})

@app.route("/clear_charge_status", methods=["POST"])
def clear_charge_status():
    """前端讀取後清除狀態"""
    global last_status
    last_status = ""
    return jsonify({"ok": True})

# DroneController is not used in this deployment. Keep the variable for API compatibility.
drone_controller = None

@app.route("/get_telemetry")
def get_telemetry():
    """統一的遙測數據端點，組合所有飛行狀態資訊"""
    data = load_data()
    batt = data.get("battery", {})
    gps = data.get("gps_position", {})
    att = data.get("attitude", {})
    
    volt = batt.get("volt", 0.0)
    min_v, max_v = 14.8, 16.8
    battery_percent = 0.0
    if volt:
        battery_percent = max(0, min(100, (volt - min_v) / (max_v - min_v) * 100))
        battery_percent = round(battery_percent, 1)
    
    return jsonify({
        "battery": battery_percent,
        "altitude": data.get("altitude", 0.0),
        "speed": data.get("speed", 0.0),
        "lat": gps.get("lat", 0.0),
        "lon": gps.get("lon", 0.0),
        "pitch": att.get("pitch", 0.0),
        "roll": att.get("roll", 0.0),
        "yaw": att.get("yaw", 0.0)
    })

@app.route("/start_charging_monitor", methods=["POST"])
def start_charging_monitor():
    if drone_controller:
        drone_controller.start_charging_monitor()
        return jsonify({"status": "ok", "msg": "已啟動充電紀錄"})
    else:
        return jsonify({"status": "fail", "msg": "無法連接 DroneController"})

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)
