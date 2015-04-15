from sys import maxint
from random import uniform
from vis import Visualizer

class Problem(object):
    def __init__(self):
        pass

    def random_state(self):
        raise NotImplementedError( "Should have implemented this" )

    def new_state(self, x1, x2):
        ''' Selects an input that would move sate x1 towards state x2 and returns 
            the new state together with the input.

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

# assumes Problem has methods:
# - random_state() -> state (tuple)
# - metric(state1, state2) -> distance (int or float)
# - new_state(state1, state2, time_step) -> state, input

class RRT(object):

    def __init__(self, problem):
        self.P = problem
        self.root = None # not currently used, but could be useful
        self.nodes = [] # list of nodes in tree

    def build_rrt(self, x_init, x_goal, max_iter=100, goal_bias=0, visualize=False):
        ''' Builds RRT, given start state, goal state, and other algorithm parameters.

            Input arguments:
            - x_init: start state
            - x_goal: goal state
            - max_iter: maximum number of iterations. default=100
            - goal_bias: probability of sampling x_goal. default=0
            - vis_steps: visualize each step as new nodes are added
            
            Returns:
            - State x such that goal_reached(x) is true.
            - None if goal state is not reached before max number of iterations.
        '''

        if visualize:                
            self.v = self.P.setup_vis()
            self.v.draw_initial(x_init)

        self.root = Node(x_init)
        self.nodes.append(self.root)

        if self.P.goal_reached(x_init):
            return x_init

        counter = 0

        while counter < max_iter:

            counter += 1

            # select a random state with probability = 1-goal_bias,
            # or the goal state with probability = goal_bias
            if uniform(0,1) >= goal_bias:
                x_rand = self.P.random_state()
            else:
                x_rand = x_goal

            # extend the tree in the direction of x_rand
            x_near, x_new = self.extend(x_rand)

            if visualize:
                self.v.draw_edge(x_near, x_new)

            if x_new and self.P.goal_reached(x_new):
                print('Reached goal in %d iterations' % counter)
                if visualize:
                    self.v.draw_solution([x.data for x in self.get_path(x_new)[0]])
                    self.v.done()
                return x_new

        # goal not reached
        print('Reached max number of iterations (%d)' % max_iter)
        if visualize:
            self.v.done()
        return None

    def nearest_neighbor(self, x):
        ''' Returns node in tree with minimum distance to x, as defined by the P.metric function.
        '''
        
        min_dist = maxint
        nearest_node = None
        for node in self.nodes:
            dist = self.P.metric(node.data, x)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def extend(self, x):
        nearest_node = self.nearest_neighbor(x)
        (x_new, u_new) = self.P.new_state(nearest_node.data, x)
        if x_new:
            self.add_node(x_new, nearest_node, u_new)
            return (nearest_node.data, x_new)
        return False

    def add_node(self, data, parent_node, edge):
        ''' Adds a node to the tree.

        Input arguments:
        - x_new: new state to be added.
        - parent_node: Node object that is parent of new node.
        - edge: input action that transitions from parent state to new state.
        '''
        new_node = Node(data, parent_node, edge)
        self.nodes.append(new_node)
        parent_node.children.append(new_node)

    def get_node(self, state):
        ''' Given a state, returns the corresponding node in the tree.

        Input arguments:
        - state: desired state. If len(state) = s, the function will search for a node such that the first s elements of node.data are equal to state.

        Returns:
        - first Node in the tree that corresponds to the input state. 
        - None if the tree doesn't have a node representing the desired state. 
        '''
        s = len(state)
        for node in self.nodes:
            if node.data[:s] == state:
                return node
        return None

    def get_path(self, x_goal):
        ''' Returns path from start state to goal state.

        Input arguments:
        - x_goal: goal state.
        
        Returns: 
        - path: list of states.
        - inputs: list of inputs to get from start state to goal state.
        '''
        node = self.get_node(x_goal)
        path = []
        inputs = []
        while node: # this should iterate until it reaches the root node
            path.append(node)
            inputs.append(node.incoming_edge)
            node = node.parent
        return path, inputs
    

class Node(object):
    
    def __init__(self, data, parent=None, incoming_edge=None):
        ''' Input arguments:
        - data: probably represented as a tuple
        - parent: Node object
        - incoming_edge: input that transitions from parent state to this state
        '''
        self.data = data
        self.parent = parent
        self.incoming_edge = incoming_edge
        self.children = [] # this isn't used for anything right now

