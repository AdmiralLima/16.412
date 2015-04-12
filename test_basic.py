import rrt
import random
import math

import vis

class BasicProblem(rrt.Problem):
	def __init__(self):
		self.dt = 0.05
		self.x_min = 0.0; self.x_max = 1.0
		self.y_min = 0.0; self.y_max = 1.0

	def random_state(self):
		x = random.uniform(self.x_min, self.x_max)
		y = random.uniform(self.y_min, self.y_max)
		return (x,y)

	def new_state(self, x1, x2):
		# get direction vector
		n = math.sqrt((x2[0]-x1[0])**2+(x2[1]-x1[1])**2)
		try:
			u = ((x2[0]-x1[0])/n, (x2[1]-x1[1])/n)
		except ZeroDivisionError:
			u = (0.0,0.0)
		# get new sate
		x = (x1[0]+u[0]*self.dt, x1[1]+u[1]*self.dt)
		return (x, u)

	def metric(self, x1, x2):
		return (x2[0]-x1[0])**2+(x2[1]-x1[1])**2

if __name__ == '__main__':
	# Problem
	problem = BasicProblem()

	# Solve
	tree = rrt.RRT(problem)
	tree.build_rrt((0.5,0.5), (1.0, 1.0), 100)

	# Visualize
	visualizer = vis.Visualizer(problem.x_min, problem.x_max, problem.y_min, problem.y_max, [])
	for n in tree.nodes:
		if n.parent:
			visualizer.draw_edge(n.parent.data, n.data)
	visualizer.draw_initial(tree.root.data)
	visualizer.done()