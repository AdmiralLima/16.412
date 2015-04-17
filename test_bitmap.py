#!/usr/bin/env python

import rrt
from problem import BitmapProblem

from PIL import Image


if __name__ == '__main__':
	# Problem
	problem = BitmapProblem(Image.open("./benchmarks/slit_500x500_offset_init80x250_goal420x250.png"), (80, 250), (420,250), 20)
	# problem = BitmapProblem(Image.open("./benchmarks/slit_500x500_centre_init80x250_goal420x250.png"), (80, 250), (420,250), 20)
	# problem = BitmapProblem(Image.open("./benchmarks/twoslits_500x500_init80x250_goal420x250.png"), (80, 250), (420,250), 20)
	# problem = BitmapProblem(Image.open("./benchmarks/maze_500x500_init147x29_goal470x430.png"), (147, 29), (470,430), 20)
	# problem = BitmapProblem(Image.open("./benchmarks/block_500x500_init80x250_goal420x250.png"), (80, 250), (420,250), 20)
	# problem = BitmapProblem(Image.open("./benchmarks/worms_500x500_init60x440_goal440x60.png"), (60, 440), (440,60), 20)
	
	# Solve
	#solver = rrt.RRT(problem)
	solver = rrt.BIRRT(problem)
	final_state,tree1,tree2 = solver.build_rrt(problem.x_init, problem.x_goal, 10000)