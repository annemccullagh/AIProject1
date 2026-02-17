"""
This code is adapted from search.py in the AIMA Python implementation, which is published with the license below:

	The MIT License (MIT)

	Copyright (c) 2016 aima-python contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Search (Chapters 3-4)

The way to use this code is to subclass Problem to create a class of problems,
then create problem instances and solve them with calls to the various search
functions."""
#imports needed for code 
import sys
import sys
from collections import deque
import heapq
from subway import SubwayMap, build_boston_map, build_london_map, straight_line_distance
#______________________________________________________________________________

'''DO NOT MODIFY THIS CLASS'''

class Problem:
	"""The abstract class for a formal problem.  You should subclass this and
	implement the method successor, and possibly __init__, goal_test, and
	path_cost. Then you will create instances of your subclass and solve them
	with the various search functions."""
	
	

	def __init__(self, initial, goal=None):
		"""The constructor specifies the initial state, and possibly a goal
		state, if there is a unique goal.  Your subclass's constructor can add
		other arguments."""
		self.initial = initial; self.goal = goal
		
	def successor(self, state):
		"""Given a state, return a sequence of (action, state) pairs reachable
		from this state. If there are many successors, consider an iterator
		that yields the successors one at a time, rather than building them
		all at once. Iterators will work fine within the framework."""
		raise NotImplementedError("successor() must be implemented in subclass")
	
	def goal_test(self, state):
		"""Return True if the state is a goal. The default method compares the
		state to self.goal, as specified in the constructor. Implement this
		method if checking against a single self.goal is not enough."""
		return state == self.goal
	
	def path_cost(self, c, state1, action, state2):
		"""Return the cost of a solution path that arrives at state2 from
		state1 via action, assuming cost c to get up to state1. If the problem
		is such that the path doesn't matter, this function will only look at
		state2.  If the path does matter, it will consider c and maybe state1
		and action. The default method costs 1 for every step in the path."""
		return c + 1
		
	def h(self, node):
		"""Return the heuristic function value for a particular node. Implement
		this if using informed (heuristic) search."""
		return 0
#______________________________________________________________________________
	
'''DO NOT MODIFY THIS CLASS'''

class Node:
	"""A node in a search tree. Contains a pointer to the parent (the node
	that this is a successor of) and to the actual state for this node. Note
	that if a state is arrived at by two paths, then there are two nodes with
	the same state.  Also includes the action that got us to this state, and
	the total path_cost (also known as g) to reach the node.  Other functions
	may add an f and h value. You will not need to
	subclass this class."""

	__nextID = 1

	def __init__(self, state, parent=None, action=None, path_cost=0):
		"Create a search tree Node, derived from a parent by an action."
		self.state = state
		self.parent = parent
		self.action = action
		self.path_cost = path_cost
		self.depth = 0
		self.id = Node.__nextID
		Node.__nextID += 1
		
		if parent:
			self.depth = parent.depth + 1
			
	def __str__(self):
		return "<Node " + str(self.state) + ">"
	
	def __repr__(self):
		return "<Node " + str(self.state) + ">"	
	
	def path(self):
		"Create a list of nodes from the root to this node."
		x, result = self, [self]
		while x.parent:
			result.append(x.parent)
			x = x.parent
		return result[::-1]

	def expand(self, problem):
		"Return a list of nodes reachable from this node. [Fig. 3.8]"
		return [Node(next, self, act,
					 problem.path_cost(self.path_cost, self.state, act, next))
				for (act, next) in problem.successor(self.state)]
	
	def __eq__(self, other):
		if isinstance(other, Node):
			return self.id == other.id
		return False
	
	def __lt__(self, other):
		if isinstance(other, Node):
			return self.id < other.id
		raise TypeError("\'<\' not supported between instances of Node and "+str(type(other)))
	
	def __hash__(self):
		return hash(self.id)

#______________________________________________________________________________

# Question 1/ (6 is also in here): Subway Navigation Problem
class SubwayNavigationProblem(Problem):
	"""A problem for finding a path between two subway stations."""
	
	def __init__(self, initial_station, goal_station, subway_map, radius=0):
		"""
		initial_station: Starting Station object
		goal_station: Goal Station object
		subway_map: SubwayMap object
		radius: Optional distance in km - consider any station within this distance as a goal
		"""
		super().__init__(initial_station, goal_station)
		self.subway_map = subway_map
		self.radius = radius
		
	def successor(self, state):
		"""Return (action, next_state) pairs for stations reachable from state."""
		successors = []
		for link in self.subway_map.incident_links(state):
			next_station = link.opposite(state)
			# action is link (contains route info)
			successors.append((link, next_station))
		return successors
	
	def goal_test(self, state):
		"""Check if state is the goal or within radius of goal."""
		if self.radius == 0:
			return state == self.goal
		else:
			# Check if state is within radius of goal(km)
			distance = straight_line_distance(state, self.goal)
			return distance <= self.radius
	
	def path_cost(self, c, state1, action, state2):
		"""Return cost of path. Action is a Link with distance."""
		return c + action.get_distance()
	
	def h(self, node):
		"""Heuristic: straight-line distance to goal station."""
		return straight_line_distance(node.state, self.goal)

'''DO NOT MODIFY THE HEADERS OF ANY OF THESE FUNCTIONS'''

def breadth_first_search(problem):
	"""Breadth-first graph search algorithm."""
	# early goal test for BFS
	node = Node(problem.initial)
	if problem.goal_test(node.state):
		return node, 1
	
	frontier = deque([node])
	explored = set()
	nodes_visited = 0
	
	while frontier:
		node = frontier.popleft()
		nodes_visited += 1
		explored.add(node.state)
		
		for child in node.expand(problem):
			if child.state not in explored and child not in frontier:
				if problem.goal_test(child.state):
					return child, nodes_visited
				frontier.append(child)
	
	return None, nodes_visited
	pass
	
def depth_first_search(problem):
	"""Depth-first graph search algorithm."""
	node = Node(problem.initial)
	frontier = [node]
	explored = set()
	nodes_visited = 0
	
	while frontier:
		node = frontier.pop()
		nodes_visited += 1
		
		if problem.goal_test(node.state):
			return node, nodes_visited
		
		explored.add(node.state)
		
		for child in node.expand(problem):
			if child.state not in explored and child not in frontier:
				frontier.append(child)
	
	return None, nodes_visited
def uniform_cost_search(problem):
	"""Uniform-cost graph search algorithm."""
	node = Node(problem.initial)
	frontier = []
	heapq.heappush(frontier, (node.path_cost, node))
	explored = set()
	nodes_visited = 0
	
	while frontier:
		_, node = heapq.heappop(frontier)
		nodes_visited += 1
		
		if problem.goal_test(node.state):
			return node, nodes_visited
		
		explored.add(node.state)
		
		for child in node.expand(problem):
			if child.state not in explored:
				# check if child in frontier
				in_frontier = False
				for i, (priority, frontier_node) in enumerate(frontier):
					if frontier_node.state == child.state:
						in_frontier = True
						# ff found with higher cost, replace 
						if child.path_cost < priority:
							frontier[i] = (child.path_cost, child)
							heapq.heapify(frontier)
						break
				
				if not in_frontier:
					heapq.heappush(frontier, (child.path_cost, child))
	
	return None, nodes_visited
#______________________________________________________________________________
# Informed (Heuristic) Search
#q5
def astar_search(problem):
	"""A* graph search algorithm."""
	node = Node(problem.initial)
	f_score = node.path_cost + problem.h(node)
	frontier = []
	heapq.heappush(frontier, (f_score, node))
	explored = set()
	nodes_visited = 0
	
	while frontier:
		_, node = heapq.heappop(frontier)
		nodes_visited += 1
		
		if problem.goal_test(node.state):
			return node, nodes_visited
		
		explored.add(node.state)
		
		for child in node.expand(problem):
			if child.state not in explored:
				child_f = child.path_cost + problem.h(child)
				
				# check if child in frontier
				in_frontier = False
				for i, (priority, frontier_node) in enumerate(frontier):
					if frontier_node.state == child.state:
						in_frontier = True
						# if higher f-score found, replace
						if child_f < priority:
							frontier[i] = (child_f, child)
							heapq.heapify(frontier)
						break
				
				if not in_frontier:
					heapq.heappush(frontier, (child_f, child))
	
	return None, nodes_visited
#______________________________________________________________________________
#7 

## Main

def main():
	'''REPLACE THIS CODE WITH CODE THAT RUNS THE PROGRAM SPECIFIED IN THE COMMAND ARGUMENTS'''

	print(sys.argv) # Prints the command line arguments. Note that the 0th element is the name of the file (search.py).


main()
