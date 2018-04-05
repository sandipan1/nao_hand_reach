import numpy as np

from gym.envs.robotics import rotations, robot_env, utils


def goal_distance(goal_a, goal_b):
	assert goal_a.shape == goal_b.shape
	return np.linalg.norm(goal_a - goal_b, axis=-1)



class NaoEnv(robot_env.RobotEnv):
	def __init__(self, model_path, n_substeps,block_finger,distance_threshold, reward_type):
		
		initial_qpos={'RElbowYaw': 0.0, 
		'RElbowRoll': 0.0,  
		'RShoulderRoll': 0.0, "RShoulderPitch":0.0}
		
		self.block_finger = block_finger   #boolean
		self.distance_threshold = distance_threshold  
		self.reward_type = reward_type      #boolean
		self.hinge=[-0.001,-0.09387237 ,0.01 ]
		super(NaoEnv, self).__init__(
			model_path=model_path, n_substeps=n_substeps, n_actions=4,
			initial_qpos=initial_qpos)
		


		# initialize all the joints with a fixed in __init__

 # ----------------------------

	def compute_reward(self, achieved_goal, goal, info):
		# Compute distance between goal and the achieved goal.
		d = goal_distance(achieved_goal, goal)
		if self.reward_type == 'sparse':
			return -(d > self.distance_threshold).astype(np.float32)
		else:
			return -d

	# RobotEnv methods
	# ----------------------------

	def _step_callback(self):
		pass
		
#right hand joint from 29 to 42 (including fingers)
# action space from 29 to 32

	def _set_action(self, action):
		assert action.shape == (4,)
		action = action.copy()  # ensure that we don't change the action outside of this scope
		action =action * 1
		for i in range(len(self.joint_name_list)):
			prev_state=self.sim.data.get_joint_qpos(self.joint_name_list[i])
			self.sim.data.set_joint_qpos(self.joint_name_list[i],prev_state+action[i])
		
		

	def _get_obs(self):
		# positions
		reach_xpos = self.sim.data.get_site_xpos('reacher')
		dt = self.sim.nsubsteps * self.sim.model.opt.timestep
		reach_xvelp = self.sim.data.get_site_xvelp('reacher') * dt
		#robot_qpos, robot_qvel = utils.robot_get_obs(self.sim)
		self.joint_name_list=self.sim.model.joint_names
		valqpos=np.array(list(map(self.sim.data.get_joint_qpos,self.joint_name_list)))
		valqvel=np.array(list(map(self.sim.data.get_joint_qvel,self.joint_name_list)))

		obj_pos=self.sim.data.get_site_xpos("target")
		obj_velp = self.sim.data.get_site_xvelp('target') * dt
		obj_velr = self.sim.data.get_site_xvelr('target') * dt
		obj_rel_pos= obj_pos-reach_xpos #- finger_xpos
			
		obs=np.concatenate([reach_xpos,obj_pos,obj_rel_pos,valqpos[0:5],reach_xvelp,obj_velp,obj_velr,valqvel[0:5]])
		
		return {
			'observation': obs.copy(),
			'achieved_goal': reach_xpos.copy(),
			'desired_goal': self.goal.copy(),
		}

	def _viewer_setup(self):
		
		self.viewer.cam.distance = 1.5
		self.viewer.cam.azimuth = 180.
		self.viewer.cam.elevation = -15.

	def _render_callback(self):
		# Visualize target.
		sites_offset = (self.sim.data.site_xpos - self.sim.model.site_pos).copy()
		site_id = self.sim.model.site_name2id('target')
		self.sim.model.site_pos[site_id] = self.goal - sites_offset[0]
		self.sim.forward()

	def _reset_sim(self):
		self.sim.set_state(self.initial_state)
		self.sim.forward()
		return True

	def _sample_goal(self):
		while (1):
			qpos = self.hinge + .5*self.np_random.uniform(low=-.2, high=.2, size=3)
			if np.linalg.norm(qpos) <= .25:
				break

		return qpos

		#goal = self.initial_gripper_xpos[:] + self.np_random.uniform(-0.15, 0.15, size=3)
		

	def _is_success(self, achieved_goal, desired_goal):
		d = goal_distance(achieved_goal, desired_goal)
		return (d < self.distance_threshold).astype(np.float32)

	def _env_setup(self, initial_qpos):
		for name, value in initial_qpos.items():
			self.sim.data.set_joint_qpos(name, value)
		self.sim.forward()

		# Move end effector into position.
		# Extract information for sampling goals.
		self.initial_gripper_xpos = self.sim.data.get_site_xpos('reacher').copy()
		