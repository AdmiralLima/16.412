import math
import numpy as np
import networkx as nx

import matplotlib.pyplot as plot

import Visualizer as vis

class Solver:
	def __init__(self, problem, visualizer=None):
		self.visualizer = visualizer
		self.problem = problem
		self.G = nx.Graph();

	def build_RRT(self, iteratrions):
		self.G.add_node(tuple(self.problem.x_init))

		for i in xrange(0, iteratrions):
			x_rand = self.problem.random_state()
			x_near = self.nearest_neighbour(x_rand)
			u = self.problem.select_input(x_near, x_rand)
			x_new = self.problem.new_state(x_near, u)

			t_new = tuple(x_new); t_near = tuple(x_near)
			self.G.add_node(t_new)
			self.G.add_edge(t_near, t_new, input=u)

			if self.visualizer:
				self.visualizer.draw_edge(t_near, t_new)

		self.visualizer.draw_initial(tuple(self.problem.x_init))

		return self.G

	def nearest_neighbour(self, x):
		min_dist  = float("inf")
		min_state = None

		for state in self.G.nodes_iter():
			dist = self.problem.metric(state,x)
			if dist < min_dist:
				min_dist  = dist
				min_state = state

		return min_state


class Problem:
	def __init__(self, x_init, x_goal):
		self.x_init = np.array(x_init)
		self.x_goal = np.array(x_goal)

	def random_state(self):
		return np.random.uniform(0,1,len(self.x_init))

	def new_state(self, x1, u):
		step = 0.05
		return x1+u*step

	def state_valid(self, x):
		return (0 <= x[0] <= 1) and (0 <= x[1] <= 1)

	# Returns a direction vector from x1 to x2
	def select_input(self, x1, x2):
		d = x2-x1
		return d/np.linalg.norm(d)

	def metric(self, x1, x2):
		return np.linalg.norm(x2-x1)

	def state_valid(self):
		return True

	def transition_valid(self, x1, x2):
		return True


if __name__ == "__main__":
	p = Problem((0.5, 0.5), (1.0, 1.0))

	v = vis.Visualizer(0, 1, 0, 1, [])

	s = Solver(p, v)

	G = s.build_RRT(100)

	v.done()
