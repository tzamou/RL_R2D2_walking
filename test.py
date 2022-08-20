import gym
env = gym.make("LunarLander-v2" )
env.reset()
action=gym.spaces.Discrete(4)

for i in range(1000):
    obs,reward,done,info = env.step(action.sample())
    if done:
        env.reset()
    env.render()




# import gym
# env = gym.make('CartPole-v1')
# env.reset()
# action=gym.spaces.Discrete(2)
#
# for i in range(1000):
#     obs,reward,done,info = env.step(action.sample())
#     if done:
#         env.reset()
#     env.render()