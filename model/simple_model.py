import numpy as np 
import matplotlib.pyplot as plt
from sys import path
path.append("../")
from controller.simple_control import CtrlPID
class SpringMass(object):
	"""docstring for SpringMass"""
	def __init__(self, sys_para, states,ts, noise=None):
		super(SpringMass, self).__init__()
		self.m = sys_para[0]
		self.k = sys_para[1]
		self.r = sys_para[2]
		self.noise = None
		self.x0 = states[0]
		self.xdot0 = states[1]
		self.ts = ts
		self.kp = 4
		self.kd = 25
		self.err1 = 0
	def dyn_equation(self, states, u):
		#xddot + r/m * xdot + k/m * x= u/m
		# f is the input
		x = states[0]
		xdot = states[1]
		xddot = (u - self.r * xdot -self.k * x) / self.m
		return xddot

	def pos_vel(self, inti_states, xddot):
		x0 = inti_states[0]
		xdot0 = inti_states[1]
		xdot = xdot0 + xddot * self.ts
		x = x0 + xdot * self.ts
		return x, xdot

	def process(self, duration, des):

		time_arr = np.arange(0,duration,self.ts)
		step_len = len(time_arr)
		x_arr = np.zeros((step_len,1))
		xdot_arr = np.zeros((step_len,1))
		x0 = self.x0;
		xdot0 = self.xdot0
		inti_states = [x0, xdot0]
		err = des - inti_states[0]
		ctrl = CtrlPID([10,21,2])
		for time in np.arange(0,step_len,1):
			x_arr[time] = inti_states[0]
			xdot_arr[time] = inti_states[1]

			#u = self.control_PID(err,inti_states)
			u = ctrl.ctrl_pid(err,self.ts)
			xddot = self.dyn_equation(inti_states,u)
			pos, vel = self.pos_vel(inti_states, xddot)
		
			inti_states = [pos, vel]
			err = des - pos
		return time_arr, x_arr, xdot_arr
	def control_PID(self, err, states):
		err_dot = (err - self.err1)/self.ts
		u = (self.k * states[0] + self.r * states[1]) + self.kp * err + self.kd * err_dot
		self.err1 = err
		return u
def test():
	ts = 0.01
	duration = 10
	states = [2,0]
	mass = 2
	r = 4
	k = 8
	sys_para = [mass, k, r]
	des = -8
	spm = SpringMass(sys_para, states, ts)
	t,x,y = spm.process(duration, des)
	plt.plot(t,x)
	#plt.ion()
	plt.show()


if __name__ == '__main__':

	test()

