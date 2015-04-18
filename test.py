import unittest
from problem import *
from rrt import *
from math import sqrt

class TestBasic2DProblem(unittest.TestCase):

	def setUp(self):
		self.p = Basic2DProblem(x_min=0., x_max=1., y_min=0., y_max=1.,
								init=(0.5, 0.5), goal= (1.0,1.0),
								goal_tolerance=0.05, max_step=0.05)

	def test_init(self):
		with self.assertRaises(AssertionError):
			Basic2DProblem(init=(1.,-1.))
		with self.assertRaises(AssertionError):
			Basic2DProblem(goal=(-1.,2.))

	def test_random_state(self):
		(x, y) = self.p.random_state()
		self.assertTrue(x >= self.p.x_min and x <= self.p.x_max, 'x out of bounds')
		self.assertTrue(y >= self.p.y_min and y <= self.p.y_max, 'y out of bounds')
		self.assertTrue(type(x)==float and type(y)==float, 'x and y should be floats. type(x)=%s, type(y)=%s' % (type(x), type(y)))

	def test_metric(self):
		self.assertEqual(self.p.metric((0., 3.), (4., 0.)), 25.0)
		self.assertEqual(self.p.metric((0., 0.3), (0., 0.0)), 0.09)

	def test_valid_state(self):
		valid_states = [(0.,1.), (0.,0.), (1.,1.), (1.,0.), (0.5, 0.5)]
		invalid_states = [(-0.01,0.01), (-0.01,-0.01), (1.01,0.5), (0.5, 1.01)]
		for state in valid_states:
			self.assertTrue(self.p.valid_state(state))
		for state in invalid_states:
			self.assertFalse(self.p.valid_state(state))

	def test_new_state(self):
		x, u = self.p.new_state((0.,0.), (1.,1.))
		self.assertEqual(x, (0.05/sqrt(2.), 0.05/sqrt(2.)))
		self.assertEqual(u, (0.05/sqrt(2.), 0.05/sqrt(2.)))

		x, u = self.p.new_state((0.,0.), (1.,1.), reverse=True)
		self.assertEqual(x, (0.05/sqrt(2.), 0.05/sqrt(2.)))
		self.assertEqual(u, (-0.05/sqrt(2.), -0.05/sqrt(2.)))

		x, u = self.p.new_state((0.,0.), (0.02,0.02))
		self.assertEqual(x, (0.02, 0.02))
		self.assertEqual(u, (0.02, 0.02))

		x, u = self.p.new_state((0.,0.), (0.02,0.02), reverse=True)
		self.assertEqual(x, (0.02, 0.02))
		self.assertEqual(u, (-0.02, -0.02))

		x, u = self.p.new_state((-1.,-1.), (2.,2.))
		self.assertEqual(x, None)
		self.assertEqual(u, None)

	def test_goal_reached(self):
		self.assertTrue(self.p.goal_reached((0.9,0.9)))
		self.assertFalse(self.p.goal_reached((0.5,0.5)))


class TestBitmapProblem(unittest.TestCase):

	def setUp(self):
		image = Image.open('./test_bitmap.png')
		self.p = BitmapProblem(image=image, init=(80,250), goal=(420,250), 
								max_step=20, goal_tolerance=5)

	def test_random_state(self):
		(x, y) = self.p.random_state()
		self.assertTrue(type(x)==int and type(y)==int, '(x,y) must be integers')
		self.assertTrue(0 <= x <= 500 and 0 <= y <= 500, 'state out of bounds')

	def test_valid_state(self):
		self.assertFalse(self.p.valid_state((250,250)))
		self.assertTrue(self.p.valid_state((50,50)))
		self.assertFalse(self.p.valid_state((501,501)))


class TestObstacleProblem(unittest.TestCase):

	def setUp(self):
		self.p = ObstacleProblem()

	def test_inside_circle(self):
		circle = ((0.,0.), 0.5)
		point1 = (0.,0.)
		point2 = (0.6,0.6)
		self.assertTrue(self.p.inside_circle(point1,circle))
		self.assertFalse(self.p.inside_circle(point2,circle))

	def test_generate_obstacles(self):
		obstacles = self.p.generate_obstacles(10,0.1)
		for o in obstacles:
			self.assertFalse(self.p.inside_circle(self.p.x_init,o), 'init inside obstacle')
			self.assertFalse(self.p.inside_circle(self.p.x_goal,o), 'goal inside obstacle')


class TestTree(unittest.TestCase):

	def setUp(self):
		self.t = Tree((0,0))

	def test_add_node(self):
		self.t.add_node((2,2), self.t.root, (1,1))
		self.assertEqual([node.data for node in self.t.nodes], [(0,0),(2,2)])
		self.assertEqual(self.t.nodes[-1].parent.data, (0,0))
		self.assertEqual(self.t.nodes[-1].incoming_edge, (1,1))

	def test_get_node(self):
		self.t.add_node((2,2), self.t.root, (1,1))
		node = self.t.get_node((2,2))
		self.assertEqual(node.data, (2,2))
		self.assertEqual(node.parent.data, (0,0))
		self.assertEqual(node.incoming_edge, (1,1))

	def test_get_path(self):
		self.t.add_node((2,2), self.t.root, (1,1))
		self.t.add_node((3,0), self.t.get_node((2,2)), (1,-2))
		self.t.add_node((1,1), self.t.get_node((2,2)), (-1,-1))
		(path, inputs) = self.t.get_path((3,0))
		self.assertEqual(path, [(0,0),(2,2),(3,0)])
		self.assertEqual(inputs, [(1,1),(1,-2)])


class TestRRTBase(unittest.TestCase):

	def setUp(self):
		p = Basic2DProblem(x_min=0., x_max=1., y_min=0., y_max=1.,
						   init=(0.5, 0.5), goal= (1.0,1.0),
						   goal_tolerance=0.05, max_step=0.05)
		self.rrt = RRTBase(p)

	def test_nearest_neighbor(self):
		tree = Tree((0,0))
		tree.add_node((5,5), tree.root, (5,5))
		tree.add_node((3,0), tree.get_node((5,5)), (-2,-5))
		tree.add_node((0,3), tree.get_node((0,0)), (0,3))

		self.assertEqual(self.rrt.nearest_neighbor(tree, (4,0)).data, (3,0))
		self.assertEqual(self.rrt.nearest_neighbor(tree, (0,2)).data, (0,3))

	def test_extend(self):
		pass
				

if __name__ == '__main__':
	unittest.main()