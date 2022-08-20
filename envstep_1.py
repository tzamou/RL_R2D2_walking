import time
import gym
import pybullet as p
from gym import spaces
import numpy as np
from src import Plane,R2D2,Target

class Step1(gym.Env):
    def __init__(self):
        physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -9.8)
        p.setRealTimeSimulation(1)

        low_action = np.array([-100, -100, -100, -100],dtype=np.float32)
        high_action = np.array([100, 100, 100, 100],dtype=np.float32)
        self.action_space = spaces.Box(low=low_action,high=high_action)
        self.observation_space = spaces.Box(low=-100,high=100,shape=(7, ), dtype=np.float32)
        Plane()
        self.r2d2 = R2D2()

        self.target=np.array([1,0,0])
        self.hitRayColor = [1, 0, 1]
        self.missRayColor = [0, 1, 0]

        self.rewardlst = []
        self.timesteps = 0
        self.testtime = 0

        self.rayFroms = np.array([1, 0, 0], dtype=np.float32)
        self.rayTos = np.array([1, 0, 1], dtype=np.float32)
        Target([1,0,0])

    def step(self, action):
        self.timesteps += 1
        action *= 10
        self.r2d2.move(action[0],action[1],action[2],action[3])#rf,rb,lf,lb
        obs = self.get_obs()
        reward=self.get_reward()
        self.done = obs[2]<0.2



        results = p.rayTestBatch(self.rayFroms, self.rayTos)
        p.removeAllUserDebugItems()
        for index, result in enumerate(results):
            if result[0] != -1:
                p.addUserDebugLine(self.rayFroms, self.rayTos, self.hitRayColor)
                reward += 2
                self.done = True
            else:
                p.addUserDebugLine(self.rayFroms, self.rayTos, self.missRayColor)

        self.rewardlst.append(reward)
        print(self.timesteps)
        if self.timesteps>250:
            reward-=2
            self.done=True
            print(self.done)

        return obs, reward, self.done, {}
    def reset(self):
        self.rewardlst = []
        self.timesteps = 0
        self.testtime += 1
        self.donec=False
        p.resetBasePositionAndOrientation(self.r2d2.r2d2ID,self.r2d2.r2d2StartPos,self.r2d2.r2d2StartOrientation)
        obs=self.get_obs()
        return obs
    def render(self, mode='human', clode=False):
        pass
    def close(self):
        p.disconnect()

    def get_obs(self):#r2d2_x,r2d2_y,r2d2_z,target_x,target_y,target_z,dis
        left = np.array(p.getLinkState(self.r2d2.r2d2ID, 0)[0])#0,4是左腳右腳關節處 所以要取平均
        right = np.array(p.getLinkState(self.r2d2.r2d2ID, 4)[0])
        pos = (left+right)/2
        obs = np.append(pos,np.array([1,0,0]))
        dis = np.linalg.norm(pos-self.target)
        obs = np.append(obs,dis)
        return obs
    def get_reward(self):
        pos=self.get_obs()[0:2]
        return -np.linalg.norm(pos-self.target[0:2])
if __name__=='__main__':
    s = Step1()