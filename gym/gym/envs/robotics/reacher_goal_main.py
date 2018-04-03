from gym import utils
from gym.envs.robotics import reacher_goal



class ReacherGoalEnv(reacher_goal.ReacherGoal):
	def __init__(self,reward_type="sparse"):
		reacher_goal.ReacherGoal.__init__(self, 'reacher.xml',reward_type=reward_type,distance_threshold=0.05,n_substeps=20)
		utils.EzPickle.__init__(self)