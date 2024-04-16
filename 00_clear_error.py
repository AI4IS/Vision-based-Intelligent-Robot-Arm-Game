import DobotDllType as dType

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

# 将dll读取到内存中并获取对应的CDLL实例
api = dType.load()
# 建立与 dobot 的连接
state = dType.ConnectDobot(api, "COM3", 115200)[0]
print("Connect status:", CON_STR[state])

if (state == dType.DobotConnect.DobotConnect_NoError):
    dType.ClearAllAlarmsState(api)  # 清楚所有报警信息
    dType.SetEndEffectorSuctionCup(api, 0, 0, isQueued=0)
    print("清空完成")
dType.DisconnectDobot(api)
