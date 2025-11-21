import serial
import time
import threading

class VoltageReader:
    def __init__(self, port: str, baud: int = 9600, timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None
        self._stop_flag = threading.Event()
        self._callback = None
        self._threshold = None

    def register_callback(self, threshold: float, callback):
        """
        註冊低電壓回呼：
        threshold: 電壓閾值 (V)
        callback: 當讀到電壓 < threshold 時呼叫，參數為讀到的電壓值
        """
        self._threshold = threshold
        self._callback = callback

    def open(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        print(f"Opened {self.port} at {self.baud}bps")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial closed")

    def start(self):
        """啟動背景執行緒不斷讀取電壓"""
        if not self.ser or not self.ser.is_open:
            self.open()
        self._stop_flag.clear()
        threading.Thread(target=self._read_loop, daemon=True).start()

    def stop(self):
        """停止讀取並關閉序列埠"""
        self._stop_flag.set()

    def _read_loop(self):
        try:
            while not self._stop_flag.is_set():
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line:
                    try:
                        voltage = float(line)
                        print(f"Voltage: {voltage:.3f} V")
                        if self._callback and self._threshold is not None:
                            if voltage < self._threshold:
                                # 呼叫使用者註冊的回呼
                                self._callback(voltage)
                    except ValueError:
                        # 忽略非數值行
                        pass
                time.sleep(0.1)
        except Exception as e:
            print(f"讀取發生錯誤: {e}")
        finally:
            self.close()
