import rrt
import random
import math
import vis

from PIL import Image

class BitmapProblem(rrt.Problem):
	def __init__(self, image, init, goal, step):
		'''
			Takes a PIL.Image.Image object as a map. 

			White pixels are interpreted as allowable states. 
		'''
		self.map = image

		self.step = step
		self.x_min = 0; self.x_max = image.size[0]
		self.y_min = 0; self.y_max = image.size[1]

		self.x_init = init
		self.x_goal = goal

	def random_state(self):
		x = random.randint(self.x_min, self.x_max)
		y = random.randint(self.y_min, self.y_max)
		return (x,y)

	def new_state(self, x1, x2, reverse=False):
		# get desired step
		u = (x2[0]-x1[0], x2[1]-x1[1])

		n = math.sqrt((x2[0]-x1[0])**2+(x2[1]-x1[1])**2)
		
		if n < self.step: # Required step is less than maximum
			u = (u[0]*n, u[1]*n)
			x = x2
		else: # Required step is too large
			u = (u[0]*self.step/n, u[1]*self.step/n)
			x = (int(x1[0]+u[0]),int(x1[1]+u[1]))

		if not self.valid_state(x):
			return None, None

		if reverse:
			u = (-u[0],-u[1])

		return x, u

	def metric(self, x1, x2):
		return (x2[0]-x1[0])**2+(x2[1]-x1[1])**2

	def collides(self, x):
		p = self.map.getpixel(x)
		if len(p) == 4:
			return p[0:3] != (255,255,55) and p[3] != 0
		else:
			return p[0:3] != (255,255,255)

	def goal_reached(self, x):
		return (x[0]-self.x_goal[0])**2 + (x[1]-self.x_goal[1])**2 < 20**2

	def valid_state(self, x):
		return (self.x_min < x[0] < self.x_max) and (self.y_min < x[1] < self.y_max) and not self.collides(x)

if __name__ == '__main__':
	# Problem
	problem = BitmapProblem(Image.open("./slit_map2.png"), (100, 200), (400,200), 10)

	# Solve
	solver = rrt.BIRRT(problem)
	final_state,tree1,tree2 = solver.build_rrt(problem.x_init, problem.x_goal, 1000)

	# Visualize
	visualizer = vis.Visualizer(problem.x_min, problem.x_max, problem.y_min, problem.y_max, problem.map)
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