import rrt
import random
import math

from vis import Visualizer


class ObstacleProblem(rrt.Problem):
	def __init__(self):
		self.dt = 0.05
		self.x_min = 0.0; self.x_max = 1.0
		self.y_min = 0.0; self.y_max = 1.0

		self.x_init = (0.5, 0.5)
		self.x_goal = (1.0, 1.0)

		self.obstacles = []
		self.generate_obstacles(10, 0.1);

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

		# Assume that if current and new states are valid then 
		# they can be reached without any collisions
		if not self.valid_state(x) or self.collides(x1) or self.collides(x):
			return (None, None)

		return (x, u)

	def metric(self, x1, x2):
		return (x2[0]-x1[0])**2+(x2[1]-x1[1])**2

	def goal_reached(self, x):
		return (x[0]-self.x_goal[0])**2 + (x[1]-self.x_goal[1])**2 < 0.05

	def valid_state(self, x):
		return (self.x_min <= x[0] <= self.x_max) and (self.y_min <= x[1] <= self.y_max)

	def generate_obstacles(self, n, radius):
		i = 0
		while i < n:
			x = random.uniform(self.x_min, self.x_max)
			y = random.uniform(self.y_min, self.y_max)
			r = radius*(random.random()/2 + 0.5)
			circle = ((x,y),r)
			if not self.inside_circle(self.x_init, circle):
				self.obstacles.append(circle)
				i += 1

	def inside_circle(self, point, circle):
		dist = (point[0]-circle[0][0])**2 + (point[1]-circle[0][1])**2 
		return dist < circle[1]*circle[1]

	def collides(self, x):
		for o in self.obstacles:
			if self.inside_circle(x, o):
				return True
		return False

	def setup_vis(self):
		v = Visualizer(self.x_min, self.x_max, self.y_min, self.y_max, self.obstacles)
		return v

if __name__ == '__main__':
	# Problem
	problem = ObstacleProblem()

	# Solve
	solver = rrt.RRT(problem)
	final_state,tree = solver.build_rrt(problem.x_init, 200)

	# Visualize
	visualizer = Visualizer(problem.x_min, problem.x_max, problem.y_min, problem.y_max, problem.obstacles)
	for n in tree.nodes:
		if n.parent:
			visualizer.draw_edge(n.parent.data, n.data)
	visualizer.draw_initial(tree.root.data)
	if final_state:
		visualizer.draw_solution([x.data for x in tree.get_path(final_state)[0]])
	visualizer.done()