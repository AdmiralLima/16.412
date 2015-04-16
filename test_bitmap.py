import rrt
import random
import math
from vis import Visualizer

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

	def new_state(self, x1, x2):
		# get direction vector
		n = math.sqrt((x2[0]-x1[0])**2+(x2[1]-x1[1])**2)
		try:
			u = ((x2[0]-x1[0])/n, (x2[1]-x1[1])/n)
		except ZeroDivisionError:
			u = (0,0)
		# get new sate
		x = (int(x1[0]+u[0]*self.step), int(x1[1]+u[1]*self.step))

		# Assume that if current and new states are valid then 
		# they can be reached without any collisions
		if not self.valid_state(x) or self.collides(x1) or self.collides(x):
			return (None, None)

		return (x, u)

	def metric(self, x1, x2):
		return (x2[0]-x1[0])**2+(x2[1]-x1[1])**2

	def collides(self, x):
		return self.map.getpixel(x)[0:3] != (255,255,255)

	def goal_reached(self, x):
		return (x[0]-self.x_goal[0])**2 + (x[1]-self.x_goal[1])**2 < 20**2

	def valid_state(self, x):
		return (self.x_min < x[0] < self.x_max) and (self.y_min < x[1] < self.y_max)

	def setup_vis(self):
		v = Visualizer(self.x_min, self.x_max, self.y_min, self.y_max, self.map)
		return v

if __name__ == '__main__':
	# Problem
	problem = BitmapProblem(Image.open("./slit_map.png"), (100, 200), (400,200), 20)

	# Solve
	solver = rrt.RRT(problem)
	final_state,tree = solver.build_rrt(problem.x_init, problem.x_goal, 500, 0.1)


