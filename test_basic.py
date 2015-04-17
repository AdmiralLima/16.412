#!/usr/bin/env python

import rrt
from problem import Basic2DProblem

if __name__ == '__main__':
	# Problem
	problem = Basic2DProblem()

	# Solve
	solver = rrt.BIRRT(problem)
	final_state, tree1, tree2 = solver.build_rrt(problem.x_init, problem.x_goal, 100, show_vis=True)
