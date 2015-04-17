#!/usr/bin/env python

import rrt
from problem import ObstacleProblem


if __name__ == '__main__':
	# Problem
	problem = ObstacleProblem()

	# Solve
	solver = rrt.RRT(problem)
	final_state,tree = solver.build_rrt(problem.x_init, problem.x_goal, 500, 0.1)
