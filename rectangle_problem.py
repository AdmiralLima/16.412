#!/usr/bin/env python

import rrt
from PIL import Image

import math, random
from problem import BitmapProblem
from vis import RectangeVisualizer
import numpy as np

class MovingRectangleProblem(BitmapProblem):
    ''' Problem with 2-D state space and no obstacles. '''

    def __init__(self, image, rwidth, rheight, init, goal, max_step, max_rot, goal_tolerance=10):
        self.rwidth = rwidth
        self.rheight = rheight

        self.max_rot = max_rot

        self.r_min = -math.pi; self.r_max = math.pi

        super(MovingRectangleProblem, self).__init__(image, init, goal, max_step, goal_tolerance)

        self.collision_grid = None
        self.__generate_collision_grid();

    def random_state(self):
        ''' Returns a state randomly selected within problem's 2D bounds. '''
        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)
        r = random.uniform(self.r_min, self.r_max)
        return (x, y, r)

    #@classmethod
    def metric(cls, x1, x2):
        ''' Default metric for 2D problem: Euclidean distance '''

        # Need to handle the angle separately
        r = (x2[2]-x1[2] + math.pi) % (2*math.pi) - math.pi
        r = r*(cls.x_max-cls.x_min)/(2*math.pi) # scale to have same weight as x dimension

        return (x2[0]-x1[0])**2+(x2[1]-x1[1])**2 + r**2


    def valid_state(self, x):
        ''' Returns True if x is within problem's 2D bounds. '''
        within_bounds = (self.x_min <= x[0] <= self.x_max) and (self.y_min <= x[1] <= self.y_max) and (self.r_min <= x[2] <= self.r_max)
        no_collision = within_bounds and not self.collides(x)

        return within_bounds and no_collision

    def pixel_collides(self, xy):
        p = self.map.getpixel(xy)
        if len(p) == 4:
            return p[0:3] != (255,255,255) and p[3] != 0
        else:
            return p[0:3] != (255,255,255)

    def __generate_collision_grid(self, sx=4, sy=8, vis=False):
        # Generate a grid of points
        def frange(start, step, n):
            r = [0]*n
            for i in range(0, n):
                r[i] = start
                start += step
            return r

        step_x = self.rwidth/(sx-1)
        step_y = self.rheight/(sy-1)
        x_coords = frange((-self.rwidth/2.0),step_x,sx)
        y_coords = frange((-self.rheight/2.0),step_y,sy)
        points = [(x, y) for x in x_coords for y in y_coords]
        
        self.collision_grid = tuple(points)

        if vis:
            import matplotlib.pyplot as plt
            plt.plot([p[0] for p in points], [p[1] for p in points], '.')
            plt.xlim(-40, 40)
            plt.ylim(-40, 40)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.show()

    def collides(self, state):
        ''' Determines whether state x is in an obstacle region, indicated as black in bitmap. '''
        xy = state[0:2]
        r = state[2]

        cosr = math.cos(r); sinr = math.sin(r)
        for gp in self.collision_grid:
            point = (gp[0]*cosr-gp[1]*sinr+xy[0], gp[0]*sinr+gp[1]*cosr+xy[1])
            if point[0] > self.x_max or point[0] < self.x_min:
                return True
            if point[1] > self.y_max or point[1] < self.y_min:
                return True
            if self.pixel_collides(point):
                return True

        return False

    def new_state(self, x1, x2, reverse=False, try_no_rotation=False):
        ''' Takes a step from x1 toward x2.

            Returns a tuple (x, u), where:
            - x is an intermediate state along the vector x2-x1, not exceeding 
            the maximum step size from x1.
            - if reverse=False, u is the input that transitions from x1 to x.
            - if reverse=True, u is the input that transitions from x to x1.

            If the state found is not valid, returns (None, None).
        '''

        # get desired step
        # Need to handle the angle separately
        if try_no_rotation:
            r = 0
        else:
            r = (x2[2]-x1[2] + math.pi) % (2*math.pi) - math.pi
        u = [x2[0]-x1[0], x2[1]-x1[1], r]

        ns = math.sqrt((x2[0]-x1[0])**2+(x2[1]-x1[1])**2)
        nr = abs(r)
        
        if ns < self.max_step: # Required step is less than maximum
            x = [x2[0], x2[1], None]
        else: # Required step is too large
            u = [u[0]*self.max_step/ns, u[1]*self.max_step/ns, r]
            x = [x1[0]+u[0],x1[1]+u[1],None]

        if nr < self.max_rot: # Required step is less than maximum
            x[2] = x2[2]
            if try_no_rotation:
                x[2] = x1[2]
        else: # Required step is too large
            u[2] = u[2]*self.max_rot/nr
            x[2] = x1[2]+u[2]

        x = tuple(x); u = tuple(u)
        if not self.valid_state(x):
            if try_no_rotation:
                return self.new_state(x1, x2, reverse=reverse, try_no_rotation=False)
            return None, None

        if reverse:
            u = (-u[0],-u[1],-u[2])

        return x, u

    def goal_reached(self, x):
        ''' Determines whether x is within the goal region. '''
        return (x[0]-self.x_goal[0])**2 + (x[1]-self.x_goal[1])**2 + (x[2]-self.x_goal[2])**2 <= self.goal_tol**2

    def setup_vis(self):
        return RectangeVisualizer(self.x_min, self.x_max, self.y_min, self.y_max, self.rwidth, self.rheight, self.map)


if __name__ == '__main__':
    # Problem
    problem = MovingRectangleProblem(Image.open("./benchmarks/worms_500x500_init60x440_goal440x60.png"), 20, 40, (60, 440, 0), (440,60, 0), 20, max_rot=math.pi/18)    
    
    # Solve
    #solver = rrt.RRT(problem)
    solver = rrt.BIRRT(problem)
    #final_state,tree1,tree2 = solver.build_rrt(problem.x_init, problem.x_goal, 1000, show_vis=True, print_debug=True)

    counts = []
    extensions = []
    t1count = []
    t2count = []
    for i in xrange(0, 500):
        #final_state,tree1 = solver.build_rrt(problem.x_init, problem.x_goal, 50000, goal_bias=0.05, show_vis=False); tree2 = None # For normal RRT
        final_state,tree1,tree2 = solver.build_rrt(problem.x_init, problem.x_goal, 50000, show_vis=False) # For BIRRT
        
        counts.append(solver._iterations_executed)
        t1count.append(len(tree1.nodes))
        if tree2:
            t2count.append(len(tree2.nodes))

    print "counts", counts
    print "t1count", t1count
    if t2count:
        print "t2count", t2count


    print "iterations:\t", np.mean(counts), np.std(counts)
    print "t1 extensions\t", np.mean(t1count), np.std(t1count)
    if t2count:
        print "t2 extensions\t", np.mean(t2count), np.std(t2count)