#!/home/benjamin/anaconda3/envs/py38/bin/python
'''
This file is verifying the translation estimation of the pose estimator, which takes in a locally referenced body velocity and integrates it additively
'''
import numpy as np
import matplotlib.pyplot as plt

class Rover:
	def __init__(self, initial_state: np.ndarray, timestep: float):
		self.state = initial_state
		self.delta_t = timestep
		self.length = 0.1
		self.width = 0.2

	def dynamics(self, linear, angular):
		x_dot = np.array(
			[ 
				linear*np.cos(self.state[2]),
				linear*np.sin(self.state[2]),
				angular
			]
		)
		return x_dot
	
	def step(self, linear, angular):
		self.state += self.dynamics(linear, angular) * self.delta_t
	
	def plot_simulation(self):
		plt.clf()
		ax = plt.axes()
		# plot the location and orientation
		rectangle = plt.Rectangle(xy=(self.state[0], self.state[1]), width=self.width, height=self.length, angle=np.rad2deg(self.state[2]))
		ax.add_patch(rectangle)
		plt.draw()
		plt.pause(0.01)


if __name__ == '__main__':
	start, stop, dt = 0, 10, 0.1
	times = np.arange(start, stop, dt)
	rover = Rover([0, 0, 0], dt)
	for t in times:
		rover.step(0.1, 0.1)
		rover.plot_simulation()

	print("Simulation Done")
	plt.show()
