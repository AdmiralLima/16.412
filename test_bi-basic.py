import rrt
import random
import math

import vis

class BasicProblem(rrt.Problem):
	def __init__(self):
		self.ds = 0.05
		self.x_min = 0.0; self.x_max = 1.0
		self.y_min = 0.0; self.y_max = 1.0

		self.x_init = (0.1, 0.1)
		self.x_goal = (0.9, 0.9)

	def random_state(self):
		x = random.uniform(self.x_min, self.x_max)
		y = random.uniform(self.y_min, self.y_max)
		return (x,y)

	def new_state(self, x1, x2, reverse=False):
		# get desired step
		u = (x2[0]-x1[0], x2[1]-x1[1])

		n = math.sqrt((x2[0]-x1[0])**2+(x2[1]-x1[1])**2)
		
		if n < self.ds: # Required step is less than maximum
			u = (u[0]*n, u[1]*n)
			x = x2
		else: # Required step is too large
			u = (u[0]*self.ds/n, u[1]*self.ds/n)
			x = (x1[0]+u[0],x1[1]+u[1])

		if not self.valid_state(x):
			return None, None

		if reverse:
			u = (-u[0],-u[1])

		return x, u


	def metric(self, x1, x2):
		return (x2[0]-x1[0])**2+(x2[1]-x1[1])**2

	def goal_reached(self, x):
		return (x[0]-self.x_goal[0])**2 + (x[1]-self.x_goal[1])**2 < 0.05

	def valid_state(self, x):
		return (self.x_min <= x[0] <= self.x_max) and (self.y_min <= x[1] <= self.y_max)

if __name__ == '__main__':
	# Problem
	problem = BasicProblem()

	# Solve
	solver = rrt.BIRRT(problem)
	final_state, tree1, tree2 = solver.build_rrt(problem.x_init, problem.x_goal, 100)

	# Visualize
	visualizer = vis.Visualizer(problem.x_min, problem.x_max, problem.y_min, problem.y_max, [])

	for tree in [tree1,tree2]:
		for n in tree.nodes:
			if n.parent:
				visualizer.draw_edge(n.parent.data, n.data)
		visualizer.draw_initial(tree.root.data)
		if final_state:
			visualizer.draw_solution([x.data for x in tree.get_path(final_state)[0]])
		else:
			print "No solution found. Try increasing the number of iterations."

	visualizer.done()

