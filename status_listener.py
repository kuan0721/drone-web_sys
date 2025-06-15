# status_listener.py
import json
import os
import paho.mqtt.client as mqtt

MQTT_BROKER   = 'test.mosquitto.org'
MQTT_PORT     = 1883
MQTT_TOPIC    = 'goodrich/drone/status'
STATUS_FILE   = 'charge_status.json'

# 建檔：存 { "status": "" }
if not os.path.exists(STATUS_FILE):
    with open(STATUS_FILE, 'w') as f:
        json.dump({"status": ""}, f)

def on_connect(client, userdata, flags, rc):
    print(f"已連線到 MQTT Broker (rc={rc})，開始訂閱 {MQTT_TOPIC}")
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    status_str = msg.payload.decode('utf-8')
    print(f"[MQTT] 收到狀態：{status_str}")

    # 將最新狀態寫入檔案
    with open(STATUS_FILE, 'w') as f:
        json.dump({"status": status_str}, f, ensure_ascii=False)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT, 60)
print("等待 MQTT 狀態訊息...")
client.loop_forever()
