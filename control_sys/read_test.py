# main.py
from control_sys.voltage_reader import VoltageReader
import time

def low_voltage_alert(v):
    # Action to take when voltage drops below the threshold
    print(f"⚠️ Low voltage alert! Detected {v:.3f} V, battery is low!")

if __name__ == "__main__":
    reader = VoltageReader(port="/dev/ttyUSB0", baud=9600)
    # Register a callback: call low_voltage_alert when voltage < 15.0 V
    reader.register_callback(threshold=15.0, callback=low_voltage_alert)
    reader.start()

    print("Start monitoring voltage. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("User interrupted. Stopping monitoring.")
        reader.stop()
