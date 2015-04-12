from sys import maxint

class RRT(object):

    def __init__(self, problem):
        self.P = problem
        self.root = None # not currently used, but could be useful
        self.nodes = [] # list of nodes in tree


    def build_rrt(self, x_init, x_goal, max_iter):
    ''' Builds RRT, given start state, goal state, and max number of iterations.

        Input arguments:
        - x_init: start state
        - x_goal: goal state
        - max_iter: maximum number of iterations
        
        Returns:
        - True if goal state is reached.
        - False if goal state is not reached before max number of iterations.
    '''
        self.root = Node(x_init)
        self.nodes.append(self.root)

        if x_init == x_goal:
            return True

        counter = 0

        while counter < max_iter:
            x_rand = P.random_state()
            x_new = self.extend(x_rand)
            if x_new == x_goal:
                return True
            counter += 1

        return False

    def nearest_neighbor(self, x):
        ''' Returns node in tree with minimum distance to x, as defined by the P.metric function.
        '''
        i = len(x)
        
        min_dist = maxint
        nearest_node = None
        for node in self.nodes:
            dist = P.metric(node.data, x)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def extend(self, x):
        nearest_node = nearest_neighbor(x)
        (x_new, u_new) = P.intermediate_state(x, nearest_node.data, dt)
        if x_new:
            self.add_node(x_new, nearest_node, u_new)
            return x_new
        return False

    def add_node(self, x_new, parent_node, edge):
        ''' Adds a node to the tree.

        Input arguments:
        - x_new: new state to be added.
        - parent_node: Node object that is parent of new node.
        - edge: input action that transitions from parent state to new state.
        '''
        new_node = Node(x_new, parent_node, edge_label)
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
    
    def __init__(self, state, parent=None, incoming_edge=None):
    ''' Input arguments:
        - state: probably represented as a tuple
        - parent: Node object
        - incoming_edge: input that transitions from parent state to this state
    '''
        self.data = data
        self.parent = parent
        self.incoming_edge = incoming_edge
        self.children = [] # this isn't used for anything right now

