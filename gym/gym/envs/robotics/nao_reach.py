from gym import utils
from gym.envs.robotics import nao_env



class NaoReach(nao_env.NaoEnv):
	def __init__(self,reward_type="sparse"):
		print(9)
		nao_env.NaoEnv.__init__(self, 'nao_hand.xml',reward_type=reward_type,distance_threshold=0.05,n_substeps=20,block_finger=True)
		utils.EzPickle.__init__(self)