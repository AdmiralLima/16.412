import matplotlib.pyplot as plot
from PIL import Image

from matplotlib.patches import Rectangle
import matplotlib as mpl
import math

def main():
	obs = [((0,0),1), ((2,2),1), ((5,5),2)]
	vis = Visualizer(0,10,0,10,obs)
	vis.done()

class Visualizer:
	def __init__(self, xmin, xmax, ymin, ymax, obstacles):
		
		## State Space Bounds ##
		self.xmin = xmin
		self.xmax = xmax
		self.ymin = ymin
		self.ymax = ymax

		## Initialize Plot ##
		self.figure = plot.gcf()
		
		plot.xlim(self.xmin,self.xmax)
		plot.ylim(self.ymin,self.ymax)
		plot.grid(True)

		## Draw Obstacles ##
		if isinstance(obstacles, (list, tuple)):
			self.obstacles = obstacles
			for obstacle in self.obstacles:
				self.figure.gca().add_artist(plot.Circle(obstacle[0],
													obstacle[1],
													color='r',
													alpha=0.3))
		elif isinstance(obstacles, Image.Image):
			plot.imshow(obstacles)

	def draw_initial(self, x):
		plot.plot((x[0]), (x[1]), 'og')

	def draw_goal(self, x):
		plot.plot((x[0]), (x[1]), 'og')

	def draw_edge(self, x1, x2, color='k'):
		# plot.arrow(x1[0], x1[1], x2[0]-x1[0], x2[1]-x1[1], length_includes_head=True, head_width=0.008*(self.xmax-self.xmin), color=color)
		plot.plot((x1[0],x2[0]), (x1[1],x2[1]), '.-'+color)

	def draw_solution(self, path, color='r'):
		for i in xrange(0, len(path)-1):
			plot.plot((path[i][0],path[i+1][0]), (path[i][1],path[i+1][1]), 'o-'+color, linewidth=3.0)

	def done(self, block=True):
		plot.show(block=block)

class RectangeVisualizer:
	def __init__(self, xmin, xmax, ymin, ymax, rwidth, rheight, obstacles):
		## State Space Bounds ##
		self.rwidth = rwidth
		self.rheight = rheight

		self.xmin = xmin
		self.xmax = xmax
		self.ymin = ymin
		self.ymax = ymax

		## Initialize Plot ##
		self.figure = plot.gcf()
		self.ax = self.figure.add_subplot(111)
		self.ax.set_xlim(self.xmin,self.xmax)
		self.ax.set_ylim(self.ymin,self.ymax)

		plot.xlim(self.xmin,self.xmax)
		plot.ylim(self.ymin,self.ymax)
		plot.grid(True)

		## Draw Obstacles ##
		if isinstance(obstacles, (list, tuple)):
			self.obstacles = obstacles
			for obstacle in self.obstacles:
				self.figure.gca().add_artist(plot.Circle(obstacle[0],
													obstacle[1],
													color='r',
													alpha=0.3))
		elif isinstance(obstacles, Image.Image):
			plot.imshow(obstacles)

	def draw_rectange(self, xy, r, w, h):
		r = r*180/math.pi
		rect = Rectangle((xy[0]-w/2.0, xy[1]-h/2.0), w, h, 0, fill=False)
		trans = mpl.transforms.Affine2D().rotate_deg_around(xy[0], xy[1], r) + self.ax.transData
		rect.set_transform(trans)
		self.ax.add_patch(rect)

	def draw_initial(self, x):
		self.draw_rectange(x[0:2], x[2], self.rwidth, self.rheight)
		plot.plot((x[0]), (x[1]), 'og')

	def draw_goal(self, x):
		self.draw_rectange(x[0:2], x[2], self.rwidth, self.rheight)
		plot.plot((x[0]), (x[1]), 'og')

	def draw_edge(self, x1, x2, color='k'):
		# plot.arrow(x1[0], x1[1], x2[0]-x1[0], x2[1]-x1[1], length_includes_head=True, head_width=0.008*(self.xmax-self.xmin), color=color)
		plot.plot((x1[0],x2[0]), (x1[1],x2[1]), '.-'+color)

	def draw_solution(self, path, color='r'):
		for i in xrange(0, len(path)-1):
			x1 = path[i]; x2 = path[i+1]
			self.draw_rectange(x1[0:2], x1[2], self.rwidth, self.rheight)
			plot.plot((x1[0],x2[0]), (x1[1],x2[1]), 'o-'+color, linewidth=3.0)

	def done(self, block=True):
		plot.show(block=block)

if __name__ == "__main__":
	main()
