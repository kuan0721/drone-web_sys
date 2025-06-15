import json
import os
import paho.mqtt.client as mqtt

MQTT_BROKER = 'test.mosquitto.org'
MQTT_PORT = 1883
MQTT_TOPIC = 'goodrich/drone/data'
DATA_FILE = 'drone_data.json'

if not os.path.exists(DATA_FILE):
    with open(DATA_FILE, 'w') as f:
        json.dump({}, f)

def on_connect(client, userdata, flags, rc):
    print(f"已連線 MQTT Broker，回傳碼: {rc}")
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    try:
        received_data = json.loads(msg.payload.decode())
        with open(DATA_FILE, 'w') as f:
            json.dump(received_data, f)
        #print(f"接收到資料: {received_data}")
    except json.JSONDecodeError:
        print("[錯誤] JSON 解析失敗")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT, 60)
print("等待 MQTT 資料...")
client.loop_forever()
