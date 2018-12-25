import numpy as np 
class CtrlPID(object):
	"""docstring for CtrlPID"""
	def __init__(self, para, filter=None):
		super(CtrlPID, self).__init__()
		self.ki = para[0]
		self.kp = para[1]
		self.kd = para[2]
		self.filter = filter
		self.init_value()
	def init_value(self):
		self.iter = 0
		self.err_ = 0
		self.ts = 0.1
		return 0
	def ctrl_pid(self, err, ts=0.01):
		err_dot = self.get_derivative(err, ts)
		self.iter = self.iter + err * ts
		u = self.ki * self.iter + self.kp + self.kd * err_dot
		self.err_ = err
		return u
	def get_derivative(self, err, ts):
		if self.filter:
			err_dot = (err - self.err_) / ts
		else:
			err_dot = (err - self.err_) / ts
		return err_dot
		pass

	def print_data(self):
		print(self.iter)


if __name__ == '__main__':
	pid = CtrlPID([1,2,3])
	pid.ctrl_pid(1)
	pid.print_data()
