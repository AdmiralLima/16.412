import random
import math
from vis import Visualizer

from PIL import Image

class Problem(object):
    ''' Interface for Problem classes. '''

    def __init__(self):
        raise NotImplementedError( "Should have implemented this" )

    def random_state(self):
        ''' Returns a random state within the allowable state space. '''
        raise NotImplementedError( "Should have implemented this" )

    def new_state(self, x1, x2, reverse=False):
        ''' Selects an input that would move state x1 towards state x2 and returns 
            the new state together with the input. The new state should be a valid
            state (in the region of allowed states).

            Input arguments:
            - x1: current state
            - x2: desired state

            Returns:
            - x: a state closer to x2 than x1. Can be null if x1 is forbidden or 
                 no such state exists.
            - u: input required to move from state x1 to x
        '''
        raise NotImplementedError( "Should have implemented this" )

    def metric(self, x1, x2):
        ''' Metric used by the solver to find nearest neighbors. '''
        raise NotImplementedError( "Should have implemented this" )

    def goal_reached(self, x):
        ''' Returns True if the state is equal to the goal state or close to it.

            It is up to the user to define what is considered close.
        '''
        raise NotImplementedError( "Should have implemented this" )

    def setup_vis(self):
        ''' Initializes and returns Visualizer with specific problem parameters.'''
        raise NotImplementedError( "Should have implemented this" )


class Basic2DProblem(Problem):
    ''' Problem with 2-D state space and no obstacles. '''

    def __init__(self, x_min=0.0, x_max=1.0, y_min=0.0, y_max=1.0, init=(0.5,0.5), goal=(1.0,1.0), max_step=0.05, goal_tolerance=0.01):

        assert x_min <= init[0] <= x_max and y_min <= init[1] <= y_max, 'initial state %s out of bounds (x: [%d,%d], y: [%d,%d])' % ((init,), x_min, x_max, y_min, y_max)
        assert x_min <= goal[0] <= x_max and y_min <= goal[1] <= y_max, 'goal state %s out of bounds (x: [%d,%d], y: [%d,%d])' % ((goal,), x_min, x_max, y_min, y_max)

        self.max_step = max_step
        self.x_min = x_min; self.x_max = x_max
        self.y_min = y_min; self.y_max = y_max

        self.x_init = init
        self.x_goal = goal
        self.goal_tol = goal_tolerance

    def random_state(self):
        ''' Returns a state randomly selected within problem's 2D bounds. '''
        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)
        return (x, y)

    @classmethod
    def metric(cls, x1, x2):
        ''' Default metric for 2D problem: Euclidean distance '''
        return (x2[0]-x1[0])**2+(x2[1]-x1[1])**2

    def valid_state(self, x):
        ''' Returns True if x is within problem's 2D bounds. '''
        return (self.x_min <= x[0] <= self.x_max) and (self.y_min <= x[1] <= self.y_max)

    def new_state(self, x1, x2, reverse=False):
        ''' Takes a step from x1 toward x2.

            Returns a tuple (x, u), where:
            - x is an intermediate state along the vector x2-x1, not exceeding 
            the maximum step size from x1.
            - if reverse=False, u is the input that transitions from x1 to x.
            - if reverse=True, u is the input that transitions from x to x1.

            If the state found is not valid, returns (None, None).
        '''

        # get desired step
        u = (x2[0]-x1[0], x2[1]-x1[1])

        n = math.sqrt((x2[0]-x1[0])**2+(x2[1]-x1[1])**2)
        
        if n < self.max_step: # Required step is less than maximum
            x = x2
        else: # Required step is too large
            u = (u[0]*self.max_step/n, u[1]*self.max_step/n)
            x = (x1[0]+u[0],x1[1]+u[1])

        if not self.valid_state(x):
            return None, None

        if reverse:
            u = (-u[0],-u[1])

        return x, u

    def goal_reached(self, x):
        ''' Determines whether x is within the goal region. '''
        return (x[0]-self.x_goal[0])**2 + (x[1]-self.x_goal[1])**2 <= self.goal_tol

    def setup_vis(self):
        ''' Returns Visualizer object initialized with 2-D bounds of problem. '''
        return Visualizer(self.x_min, self.x_max, self.y_min, self.y_max, [])


class BitmapProblem(Basic2DProblem):
    ''' Problem with 2-D state space, using a black-and-white bitmap where black denotes obstacle regions and white denotes free regions. '''

    def __init__(self, image, init, goal, max_step, goal_tolerance=20):
        ''' Takes a PIL.Image.Image object as a map.

            White pixels are interpreted as allowable states.
        '''
        assert isinstance(image, Image.Image), "bitmap must be a PIL.Image.Image"
        self.map = image.convert('1')

        assert self.map.getpixel(init) != 0, "initial state is in an obstacle"
        assert self.map.getpixel(goal) != 0, "goal state is in an obstacle"

        super(BitmapProblem, self).__init__(0, image.size[0]-1, 0, image.size[1]-1, init, goal, max_step, goal_tolerance)

    def random_state(self):
        ''' Return a random state within 2-D bounds (i.e. pixel in image). ''' 
        x = random.randint(self.x_min, self.x_max)
        y = random.randint(self.y_min, self.y_max)
        return (x, y)

    def new_state(self, x1, x2, reverse=False):
        return super(BitmapProblem, self).new_state(x1, x2, reverse)

    def valid_state(self, x):
        ''' Determine whether state x is within bounds and not in an obstacle region. '''
        return (super(BitmapProblem, self).valid_state(x) and self.map.getpixel(x) != 0)

    def setup_vis(self):
        return Visualizer(self.x_min, self.x_max, self.y_min, self.y_max, self.map)


class ObstacleProblem(Basic2DProblem):
    ''' Problem with 2-D state space and circular obstacles. TODO: support obstacles with other shapes. '''

    def __init__(self, x_min=0.0, x_max=1.0, y_min=0.0, y_max=1.0, init=(0.5, 0.5), goal=(1.0, 1.0), max_step=0.05, goal_tolerance=0.01, obstacles=None):
        ''' Takes obstacles as a list of circles represented as ((x_center, y_center), radius). '''

        super(ObstacleProblem, self).__init__(x_min, x_max, y_min, y_max, init, goal, max_step, goal_tolerance)

        if obstacles:
            self.obstacles = obstacles
        else:
            self.obstacles = self.generate_obstacles(10, 0.1)

    def generate_obstacles(self, n, max_radius):
        ''' Returns a list of n randomly generated circles represented as ((x_center, y_center), radius). '''
        obstacles = []
        i = 0
        while i < n:
            (x, y) = self.random_state()
            r = max_radius*(random.random()/2 + 0.5)
            circle = ((x,y),r)
            if not self.inside_circle(self.x_init, circle):
                if not self.inside_circle(self.x_goal, circle):
                    obstacles.append(circle)
                    i += 1
        return obstacles

    @staticmethod
    def inside_circle(point, circle):
        ''' Determines whether point is inside circle. '''
        dist = (point[0] - circle[0][0])**2 + (point[1] - circle[0][1])**2
        return dist < circle[1]**2

    def collides(self, x):
        ''' Determines whether state x is inside an obstacle region. '''
        for o in self.obstacles:
            if self.inside_circle(x, o):
                return True
        return False

    def valid_state(self, x):
        ''' Determines whether state x is within state space bounds and not within an obstacle region. '''
        return (super(ObstacleProblem, self).valid_state(x) and not self.collides(x))

    def setup_vis(self):
        return Visualizer(self.x_min, self.x_max, self.y_min, self.y_max, self.obstacles)

