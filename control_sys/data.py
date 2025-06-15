import threading
import time
import json
import paho.mqtt.client as mqtt

class DataMonitor:
    def __init__(self, drone_controller, broker='test.mosquitto.org', port=1883, topic='goodrich/drone/data'):
        self.drone = drone_controller
        self.client = mqtt.Client()
        self.broker = broker
        self.port = port
        self.topic = topic

    def start(self):
        self.client.connect(self.broker, self.port, 60)
        threading.Thread(target=self.fetch_and_publish, daemon=True).start()

    def fetch_and_publish(self):
        while True:
            try:
                hud = self.drone.master.recv_match(type='VFR_HUD', blocking=True, timeout=2)
                speed = round(hud.groundspeed, 2) if hud else 0
                alt_msg = self.drone.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
                altitude = round(alt_msg.relative_alt/1000.0, 2) if alt_msg else 0
                sys_msg = self.drone.master.recv_match(type='SYS_STATUS', blocking=True, timeout=2)

                data = {
                    'flight_mode': self.drone.master.flightmode,
                    'arm_status': self.drone.get_arm_status(),
                    'speed': speed,
                    'altitude': altitude,
                    'battery': {
                        'volt': round(sys_msg.voltage_battery/1000.0, 2),
                        'current': None,
                        'battery_percent': sys_msg.battery_remaining
                    },
                    'gps_position': {
                        'lat': self.drone.get_gps_position()[0],
                        'lon': self.drone.get_gps_position()[1]
                    },
                    'attitude': {
                        'pitch': self.drone.get_attitude()[0],
                        'yaw': self.drone.get_attitude()[1],
                        'roll': self.drone.get_attitude()[2]
                    }
                }
                self.client.publish(self.topic, json.dumps(data))
            except Exception as e:
                print("Publish error:", e)
            time.sleep(1)