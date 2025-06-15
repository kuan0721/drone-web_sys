def check_battery(*args, **kwargs):
    """禁用電池檢查，不觸發 LowBatteryResumeException。"""
    return


def decrease_battery(amount=0):
    """禁用電量減少模擬。"""
    return


def simulate_charging(*args, **kwargs):
    """禁用充電模擬。"""
    print("實際電池測試中，跳過充電模擬。")
    return