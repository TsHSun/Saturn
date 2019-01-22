
import numpy as np 
import matplotlib.pyplot as plt
class SuspentionModel(object):
	"""docstring for SuspentionModel"""
	def __init__(self, mass=1, k=4, r=4):
		super(SuspentionModel, self).__init__()
		self.k = k
		self.m = mass
		self.r = r
		self.deg2pi = np.pi / 180
		self.init_time(0.01,10)
	def init_time(self,delta_time, max_time):
		self.delta_T = delta_time
		self.max_T = max_time
		self.time_seq = np.arange(0,self.max_T, self.delta_T)
		self.t_len = np.size(self.time_seq)
	def init_dist(self, x):
		self.x_dist_seq = x * np.sin(2 * np.pi * self.time_seq)

	def step(self,x_states, ddotx_dist, u):
		x = x_states[0]
		dotx = x_states[1]
		Fk = -self.k * x
		Fr = -self.r * dotx
		F_dist = - self.m * ddotx_dist

		ddotx = (Fk + Fr + F_dist) / self.m
		return ddotx

	def get_new_states(self,x_states0,ddotx):
		x0 = x_states0[0]
		dotx0 = x_states0[1];

		dotx1 = dotx0 + ddotx * self.delta_T
		x1 = x0 + (dotx0 + dotx1) * self.delta_T / 2

		return x1, dotx1
	def process(self, x_states0):
		x0 = x_states0[0]
		dotx0 = x_states0[1]
		x_states0 = np.array([x0, dotx0])
		u = 0
		ddotx_seq = np.zeros([self.t_len,1])
		dotx_seq = np.zeros_like(ddotx_seq)
		x_seq = np.zeros_like(dotx_seq)
		for i in range(self.t_len):
			ddotx0 = self.step(x_states0, self.x_dist_seq[i], u)
			x1, dotx1 = self.get_new_states(x_states0, ddotx0)
			x_seq[i] = x1
			dotx_seq[i] = dotx1
			ddotx_seq[i] = ddotx0

			x_states0 = np.array([x1, dotx1])

		return self.time_seq, x_seq, dotx_seq, ddotx_seq

def test():
	model = SuspentionModel()

	model.init_dist(1)
	x_states0 = [0,0]
	t,x,dx,ddx = model.process(x_states0)

	plt.plot(t,x)
	plt.show()

if __name__ == '__main__':
	test()
