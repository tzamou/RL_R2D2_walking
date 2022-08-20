import pybullet as p
import pybullet_data
import numpy as np
class Plane:
    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeId = p.loadURDF("plane.urdf")

class R2D2:
    def __init__(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.r2d2StartPos = [0, 0, 0.5]
        self.r2d2StartOrientation = p.getQuaternionFromEuler([0, 0, 3*np.pi/2])
        self.r2d2ID = p.loadURDF("r2d2.urdf",self.r2d2StartPos, self.r2d2StartOrientation)
        self.numJoints = p.getNumJoints(self.r2d2ID)
        self.targetVel = 0
        self.maxForce = 100
        self.Pmode = p.POSITION_CONTROL


    def move(self,rf,rb,lf,lb):#left front,right front,left back,right back
        p.setJointMotorControl2(self.r2d2ID, jointIndex=2, controlMode=p.VELOCITY_CONTROL, targetVelocity=rf)
        p.setJointMotorControl2(self.r2d2ID, jointIndex=3, controlMode=p.VELOCITY_CONTROL, targetVelocity=rb)
        p.setJointMotorControl2(self.r2d2ID, jointIndex=6, controlMode=p.VELOCITY_CONTROL, targetVelocity=lf)
        p.setJointMotorControl2(self.r2d2ID, jointIndex=7, controlMode=p.VELOCITY_CONTROL, targetVelocity=lb)

class Target:
    def __init__(self,pos=[1, 0, 0]):
        self.targetStartPos = pos
        self.targetStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.targetID = p.loadURDF("target.urdf", self.targetStartPos, self.targetStartOrientation)
class Wall:
    def __init__(self,pos=[0, 0, 0]):
        self.wallStartPos = pos
        self.wallStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.wallID = p.loadURDF("wall.urdf", self.wallStartPos, self.wallStartOrientation)

if __name__=='__main__':
    import time
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("plane.urdf")
    t = Target()
    # w = Wall()
    while True:
        time.sleep(1 / 240)

