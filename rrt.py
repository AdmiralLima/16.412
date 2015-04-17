from sys import maxint
from random import uniform
from vis import Visualizer

''' Module rrt provides a framework for RRT path planners. To use an RRT, initialize it with a Problem instance, and then call build_rrt(...). 
'''


class RRTBase(object):
    ''' Abstract base class for RRT solvers. Provides standard implementations of extend(), nearest_neighbor(), and visualize(). Derived classes must implement method build_rrt(). '''

    def __init__(self, problem):
        ''' Initializes RRT with a Problem object. '''
        self.P = problem

        self._iterations_executed = 0

    def build_rrt(self, x_init, x_goal, max_iter, goal_bias, show_vis=False):
        ''' Abstract method. Builds RRT, given start state, goal state, and algorithm parameters.

            Parameters:
            - max_iter: maximum number of iterations before terminating.
            - goal_bias: probability of using goal state as random state in any given iteration.
            - show_vis: boolean flag whether or not to show visualization.
        '''
        raise NotImplementedError("Should have implemented this")

    def extend(self, tree, x, reverse=False):
        ''' Extends tree in direction of state x:
            1. Finds tree's nearest neighbor to x.
            2. Finds an intermediate state that extends tree toward x.
            3. Adds new node and edge to tree and returns the new state.
               If reverse=False, the edge (input) goes from the nearest neighbor to the new node.
               If reverse=True, the edge (input) goes from the new node to the nearest neighbor.
               If a new state was not found, returns None.
        '''
        nearest_node = self.nearest_neighbor(tree, x)

        (x_new, u_new) = self.P.new_state(nearest_node.data, x, reverse=reverse)

        if x_new:
            tree.add_node(x_new, nearest_node, u_new)
            return x_new

        return None

    def nearest_neighbor(self, tree, x):
        ''' Returns node in tree with minimum distance to x, as defined by the P.metric function. '''
        min_dist = maxint
        nearest_node = None
        for node in tree.nodes:
            dist = self.P.metric(node.data, x)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def visualize(self, tree, final_state=None, x_goal=None, color='r', show=True):
        ''' Creates and displays visualization for a (solved) RRT. '''
        v = self.P.setup_vis()
        for n in tree.nodes:
            if n.parent:
                v.draw_edge(n.parent.data, n.data, color=color)
        if final_state:
            v.draw_solution([x.data for x in tree.get_path(final_state)[0]], color=color)
        v.draw_initial(tree.root.data)
        if x_goal:
            v.draw_goal(x_goal)
        if show:
            v.done()


class RRT(RRTBase):
    ''' Basic RRT implementation. '''

    def build_rrt(self, x_init, x_goal, max_iter=100, goal_bias=0, show_vis=False, print_debug=False):
        ''' Builds RRT, given start state, goal state, and other algorithm parameters.

            Input arguments:
            - x_init: start state
            - x_goal: goal state
            - max_iter: maximum number of iterations before terminating
            - goal_bias: probability of sampling x_goal
            - vis_steps: visualize each step as new nodes are added
            
            Returns:
            - State x such that goal_reached(x) is true.
            - None if goal state is not reached before max number of iterations.
        '''
        self._iterations_executed = 0 

        tree = Tree(x_init)

        if self.P.goal_reached(x_init):
            return x_init, tree

        counter = 0
        while counter < max_iter:
            counter += 1
            self._iterations_executed += 1

            # select a random state with probability = 1-goal_bias,
            # or the goal state with probability = goal_bias
            if uniform(0,1) >= goal_bias:
                x_rand = self.P.random_state()
            else:
                x_rand = x_goal

            # extend the tree in the direction of x_rand
            x_new = self.extend(tree, x_rand)

            if x_new and self.P.goal_reached(x_new):
                if print_debug:
                    print('Reached goal in %d iterations' % counter)
                    print('Final state: %s' % (x_new,))
                
                if show_vis:
                    self.visualize(tree, x_new, x_goal=x_goal)

                return x_new, tree

        # goal not reached
        if print_debug:
            print('Reached max number of iterations (%d)' % max_iter)

        if show_vis:
            self.visualize(tree, x_goal=x_goal)

        return None, tree


class BIRRT(RRTBase):
    ''' Bidirectional RRT solver. '''

    def build_rrt(self, x_init, x_goal, max_iter, show_vis=False, print_debug=False):
        ''' Builds bidirectional RRT, given start state, goal state, and max number of iterations.

            Input arguments:
            - x_init: start state
            - x_goal: goal state
            - max_iter: maximum number of iterations before terminating
            - show_vis: boolean flag whether or not to show visualization
            
            Returns:
            - State x such that goal_reached(x) is true.
            - None if goal state is not reached before max number of iterations.
        '''
        t_init = Tree(x_init)
        t_goal = Tree(x_goal)

        self._iterations_executed = 0 

        counter = 0
        t1 = t_init
        t2 = t_goal
        reverse=False
        while counter < max_iter:
            counter += 1
            self._iterations_executed += 1

            x_rand = self.P.random_state()
            
            # extends tree 1 toward random state
            x_new1 = self.extend(t1, x_rand, reverse)
            if x_new1 is not None:
                # extends tree 2 toward new state just added to tree 1
                x_new2 = self.extend(t2, x_new1, not reverse)

                if x_new1 == x_new2:
                    if print_debug:
                        print('Reached goal in %d iterations' % counter)
                    if show_vis:
                        self.visualize(t1, t2, x_new1)
                    return x_new1, t_init, t_goal

            t1, t2 = t2, t1
            reverse = not reverse
            

        # max iterations reached before trees connected
        if print_debug:
            print('Reached max number of iterations (%d)' % max_iter)
        if show_vis:
            self.visualize(t1, t2)

        return None, t_init, t_goal

    def visualize(self, tree1, tree2, final_state=None):
        super(BIRRT, self).visualize(tree1, final_state, show=False)
        super(BIRRT, self).visualize(tree2, final_state, color='b')
    

class Node(object):
    ''' Implementation of tree nodes. Each node stores:
        - data: the state that the node represents (expected as a tuple)
        - parent: a single parent node
        - incoming_edge: the input that transitions from the parent state to this state
    '''
    
    def __init__(self, data, parent=None, incoming_edge=None):
        ''' Input arguments:
        - data: state, probably represented as a tuple
        - parent: Node object
        - incoming_edge: input that transitions from parent state to this state
        '''
        self.data = data
        self.parent = parent
        self.incoming_edge = incoming_edge
        self.children = [] # this isn't used for anything right now


class Tree(object):
    ''' Basic tree implementation. ''' 

    def __init__(self, root):
        self.root = Node(root) # not currently used, but could be useful
        self.nodes = [self.root] # list of nodes in tree

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
