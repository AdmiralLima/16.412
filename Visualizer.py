import matplotlib.pyplot as plot

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
		figure = plot.gcf()
		plot.xlim(self.xmin,self.xmax)
		plot.ylim(self.ymin,self.ymax)
		plot.grid(True)

		## Draw Obstacles ##
		self.obstacles = obstacles
		for obstacle in self.obstacles:
			figure.gca().add_artist(plot.Circle(obstacle[0],
												obstacle[1],
												color='r',
												alpha=0.3))

	def done(self):
		plot.show()

if __name__ == "__main__": main()
