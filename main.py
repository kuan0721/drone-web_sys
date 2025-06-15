from control_sys.drone_control import DroneController, LowBatteryResumeException
import signal
import time

def main():
    drone = DroneController('/dev/ttyACM1', voltage_port='/dev/ttyUSB0', voltage_threshold=15.2)
    drone.connect()
    
    # 設定信號處理器
    def signal_handler(sig, frame):
        print("\n🛑 接收到中斷信號，執行緊急降落...")
        drone.return_to_launch()
    
    signal.signal(signal.SIGINT, signal_handler)

    try:
        drone.arm_and_takeoff()
        waypoints = [(drone.square_size,0), (drone.square_size,drone.square_size), (0,drone.square_size), (0,0)]
        
        for lap in range(3):
            print(f"🚁 開始第 {lap+1}/3 圈飛行")
            for i, wp in enumerate(waypoints):
                drone.next_waypoint = wp
                print(f"   導航至航點 {i+1}/4: ({wp[0]}, {wp[1]})")
                try:
                    drone.fly_to_point(*wp, drone.takeoff_altitude)
                except LowBatteryResumeException:
                    print("⚡ 低電量充電流程已完成，繼續執行任務")
                    break
            print(f"✅ 第 {lap+1} 圈完成")
        
        print("🎯 任務完成，執行最終降落")
        drone.final_rtl()
        
    except Exception as e:
        print(f"❌ 任務執行時發生錯誤: {e}")
        drone.emergency_rtl()

if __name__ == '__main__':
    main()