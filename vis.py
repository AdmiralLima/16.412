import matplotlib.pyplot as plot
from PIL import Image

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

	def draw_edge(self, x1, x2):
		plot.plot((x1[0],x2[0]), (x1[1],x2[1]), '.-k')

	def done(self, block=True):
		plot.show(block=block)

if __name__ == "__main__":
	main()
