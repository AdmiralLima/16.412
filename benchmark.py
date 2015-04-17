#!/usr/bin/env python

import rrt
import numpy as np
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

	counts = []
	extensions = []
	t1count = []
	t2count = []
	for i in xrange(0, 500):
		final_state,tree1,tree2 = solver.build_rrt(problem.x_init, problem.x_goal, 50000, show_vis=False)
		
		counts.append(solver._iterations_executed)
		t1count.append(len(tree1.nodes))
		t2count.append(len(tree2.nodes))

	print "counts", counts
	print "t1count", t1count
	print "t2count", t2count


	print "iterations:\t", np.mean(counts), np.std(counts)
	print "t1 extensions\t", np.mean(t1count), np.std(t1count)
	print "t2 extensions\t", np.mean(t2count), np.std(t2count)