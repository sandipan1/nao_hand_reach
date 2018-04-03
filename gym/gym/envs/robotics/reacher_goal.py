import numpy as np
from gym import utils
from gym.envs.robotics import rotations, robot_env, utils

def goal_distance(goal_a, goal_b):
	assert goal_a.shape == goal_b.shape
	return np.linalg.norm(goal_a - goal_b, axis=-1)


class ReacherGoal(robot_env.RobotEnv):
	def __init__(self, model_path,reward_type,distance_threshold,n_substeps=10):
		self.distance_threshold=distance_threshold
		self.reward_type=reward_type
		#self.num_action=2
		initial_qpos=np.array([0.0,0.0,0.0,0.0])

		super(ReacherGoal, self).__init__(
            model_path=model_path, n_substeps=n_substeps, n_actions=2,
            initial_qpos=initial_qpos)

	def compute_reward(self, achieved_goal, goal, info):

		# Compute distance between goal and the achieved goal.
		d = goal_distance(achieved_goal,goal)
		if self.reward_type == 'sparse':
			return -(d > self.distance_threshold).astype(np.float32)
		else:
			return -d


	def _step_callback(self):
		'''can be called after each step of simulation'''
		

	def _set_action(self, action):
		assert action.shape == (2,)
		action = action.copy()  # ensure that we don't change the action outside of this scope
		pos_ctrl = action[:2]   # robot:body0 and robot:body1 for moving limbs

		pos_ctrl *= 0.05  # limit maximum change in position

		# Apply action to simulation.
		utils.ctrl_set_action(self.sim, action)
		#utils.mocap_set_action(self.sim, action)

	def _get_obs(self):
		# positions
		finger_xpos =self.sim.data.get_body_xpos("robot:fingertip")
		dt = self.sim.nsubsteps * self.sim.model.opt.timestep
		finger_xvelp = self.sim.data.get_body_xvelp('robot:fingertip') * dt
		a=self.sim.model.joint_names
		valqpos=np.array(list(map(self.sim.data.get_joint_qpos,a)))
		valqvel=np.array(list(map(self.sim.data.get_joint_qvel,a)))
		valqvel=valqvel * dt
		obj_pos=self.sim.data.get_body_xpos("target")
		obj_velp = self.sim.data.get_body_xvelp('target') * dt
		obj_velr = self.sim.data.get_body_xvelr('target') * dt
		obj_rel_pos= obj_pos - finger_xpos

		obs=np.concatenate([finger_xpos,obj_pos,obj_rel_pos,valqpos[:2],finger_xvelp,obj_velp,obj_velr,valqvel[:2]])


		return {
			'observation': obs.copy(),
			'achieved_goal': finger_xpos[:2].copy(),
			'desired_goal': self.goal.copy(),
		}
	def _render_callback(self):
	    # Visualize target.
	    #sites_offset = (self.sim.data.site_xpos - self.sim.model.site_pos).copy()
	    #site_id = self.sim.model.site_name2id('target0')
	    #self.sim.model.site_pos[site_id] = self.goal - sites_offset[0]
	    
	    self.sim.forward()

	def _viewer_setup(self):
		
		self.viewer.cam.trackbodyid = 0

   
	def _reset_sim(self):
		self.sim.set_state(self.initial_state)
		return True
# radius =0.2
	def _sample_goal(self):
		while (1):
			qpos = self.initial_state.qpos[-2:] + 200*self.np_random.uniform(low=-.2, high=.2, size=2)
			if np.linalg.norm(qpos) <= .2:
				break

		return qpos



	def _is_success(self, achieved_goal, desired_goal):
		d = goal_distance(achieved_goal, desired_goal)
		return (d < self.distance_threshold).astype(np.float32)

	def _env_setup(self, initial_qpos):
		pass



