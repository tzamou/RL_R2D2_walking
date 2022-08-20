import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
r2d2StartPos = [0, 0, 0.5]
r2d2StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
r2d2 = p.loadURDF('r2d2.urdf', r2d2StartPos, r2d2StartOrientation)
# numJoints = p.getNumJoints(r2d2)
# for joint in range(numJoints):
#  with open('par.txt','a') as fp:
#    fp.write(str(p.getJointInfo(r2d2, joint)))
#    fp.write('\n')
wheel = p.addUserDebugParameter(paramName='wheel', rangeMin=-50, rangeMax=50, startValue=0)
btn = p.addUserDebugParameter(paramName="btn", rangeMin=1, rangeMax=0, startValue=0)
btn_value = p.readUserDebugParameter(btn)
# 上帝視角的設定
pitch = p.addUserDebugParameter(paramName='camerapitch', rangeMin=0, rangeMax=360, startValue=0)
yaw = p.addUserDebugParameter(paramName='camerayaw', rangeMin=0, rangeMax=360, startValue=30)
distance = p.addUserDebugParameter(paramName='cameradistance', rangeMin=0, rangeMax=6, startValue=2)
# 設定四顆輪子的索引值串列
wheel_lst = [2, 3, 6, 7]


def getKeyPressed():
    events = p.getKeyboardEvents()
    print(f'events:{events}')
    list_key = list(events.keys())
    print(list_key)
    return list_key
while True:
    # 讀取輪子的數值
    wheel_value = p.readUserDebugParameter(wheel)
    # p.setJointMotorControl2(r2d2, jointIndex=2, controlMode=p.VELOCITY_CONTROL, targetVelocity=wheel_value)
    # p.setJointMotorControl2(r2d2, jointIndex=3, controlMode=p.VELOCITY_CONTROL, targetVelocity=wheel_value)
    # p.setJointMotorControl2(r2d2, jointIndex=6, controlMode=p.VELOCITY_CONTROL, targetVelocity=wheel_value)
    # p.setJointMotorControl2(r2d2, jointIndex=7, controlMode=p.VELOCITY_CONTROL, targetVelocity=wheel_value)
    wheel_values = [wheel_value for i in range(len(wheel_lst))]
    p.setJointMotorControlArray(r2d2, jointIndices=wheel_lst, controlMode=p.VELOCITY_CONTROL,targetVelocities=wheel_values)

    # 如果按下按鈕則重新生成r2d2
    if p.readUserDebugParameter(btn) != btn_value:
        p.resetBasePositionAndOrientation(r2d2, r2d2StartPos, r2d2StartOrientation)
        btn_value = p.readUserDebugParameter(btn)

    # 設定上帝視角
    r2d2pos = p.getBasePositionAndOrientation(r2d2)[0]
    cameraDistance = p.readUserDebugParameter(distance)
    cameraYaw = p.readUserDebugParameter(yaw)
    cameraPitch = p.readUserDebugParameter(pitch)
    p.resetDebugVisualizerCamera(
        cameraDistance=cameraDistance,
        cameraYaw=cameraYaw,
        cameraPitch=cameraPitch,
        cameraTargetPosition=r2d2pos
    )

    # 設定雷射偵測
    raystartpos = [0, 1, 0]
    raytopos = [0, 1, 1]
    ray = p.rayTest(raystartpos, raytopos)[0][0]
    if ray == -1:
        p.addUserDebugLine(raystartpos, raytopos, [1, 0, 0])
    else:
        p.addUserDebugLine(raystartpos, raytopos, [0, 1, 0])
    p.removeAllUserDebugItems()
    # getKeyPressed()
    print(p.getMouseEvents())

