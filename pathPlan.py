#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import re
import os
import math
import numpy
import random
import time
import profile
profile = profile.Profile()


####################
# Configuration Parameters

DEBUG = False # enable debug view every X steps

PROFILE = False # enable hot spot profiler

#random.seed(2678900629721256704L) # uncomment to make simulations repeatable

p = .2 # move p% of the distance each step, 0 < p <= 1

INCLUDE_PATH_COST = False # compute path costs, conflicts with KDTree option, do not use

NEAREST_NEIGHBOR_USES_KDTREE = True # using KDTree of Naive to find nearest neighbor

OVERSAMPLE_GOAL = 32 # sample in goal every X steps

#ENVIRONMENT = "sample_environment"
ENVIRONMENT = "small2" # environment name to run, must match folder in current directory

REPEAT = 10 # number of times to average over

####################
# Execution main blocks

def main():
	print "Environment ", ENVIRONMENT
	orig_dir = os.getcwdu()
	os.chdir(ENVIRONMENT)
	box = Box("box.txt")
	if DEBUG: print "box", box.x, box.y, box.l, box.w, box.x1, box.x2, box.y1, box.y2
	init = Init("init.txt")
	obstacles = Obstacles("obstacles.txt",init.r)
	goal = Goal("goal.txt")
	if DEBUG: plotEnviron(box,init,obstacles,goal,None,None)
	totaltime = 0
	for i in xrange(REPEAT):
		tree = LocationNode((init.x, init.y), None)
		t1 = time.time()
		if PROFILE:
			node = profile.runcall(pathSearch,box,init,obstacles,goal,tree)
		else:
			node = pathSearch(box,init,obstacles,goal,tree)
		t2 = time.time() - t1
		print "Took: ", t2*1e3 , " ms"
		totaltime += t2
		if PROFILE:
			profile.print_stats(1)
	print "Average time of N runs (ms, #): ", totaltime / REPEAT * 1e3, REPEAT
	plotEnviron(box,init,obstacles,goal,tree,node)
	os.chdir(orig_dir)
	return (box, init, obstacles, goal, tree, node)

def plotEnviron(box,init,obstacles,goal,tree,node):
	ax = plt.axes()
	ax.add_patch(patches.Circle((goal.x,goal.y),goal.r,color='blue'))
	ax.add_patch(patches.Circle((init.x,init.y),init.r,color='green'))
	for ob in obstacles.list:
		ax.add_patch(patches.Circle((ob.x,ob.y),ob.r,color='black'))
	if tree:
		plotTree(ax, tree.child)
	if node:
		plotReverseTree(ax, node)
		ax.add_patch(patches.Circle((node.x,node.y),init.r,color='green'))
	ax.set_xlim(box.x1, box.x2)
	ax.set_ylim(box.y1, box.y2)
	ax.axis('scaled')
	plt.show()

# Show the tree of collision free paths
def plotTree(ax, location):
	parent = location.parent
	ax.plot((parent.x,location.x),(parent.y,location.y),color='red')
	if location.child: plotTree(ax, location.child)
	if location.sibling: plotTree(ax, location.sibling)

# Show the solution
def plotReverseTree(ax, location):
	while location.parent:
		parent = location.parent
		ax.plot((parent.x,location.x),(parent.y,location.y),color='green')
		location = location.parent

####################
# Primary algorithms

# RRT search
def pathSearch(box,init,obstacles,goal,tree):
	node = None
	samp_step = 0
	while node not in goal:
		samp_step += 1
		node = addNode(obstacles, tree, getSample(box, goal, samp_step))
		if DEBUG and (samp_step % DEBUG == 0): plotEnviron(box,init,obstacles,goal,tree,node)
	if DEBUG: print "Sample Steps: ", samp_step
	return node

# Add add node based on "sample", if feasible
def addNode(obstacles, tree, sample):
	dist, nearest = nearestNeighbor(sample, tree)
	dnew = (nearest.x*(1-p) + sample[0]*p, nearest.y*(1-p)+sample[1]*p)
	if checkCollisions(Line((nearest.x, nearest.y), dnew), obstacles):
		return None
	return LocationNode(dnew, nearest)

# Check for collision by iterating against all obstacles
def checkCollisionsNaive(line, obstacles):
	for obstacle in obstacles.list:
		if intersects(line, obstacle):
			return True
	return False

checkCollisions = checkCollisionsNaive

# Check for collision-free by checking distances
def intersectsSimple(line,circle):
	d2 = math.hypot(circle.x-line.x2, circle.y-line.y2)
	if d2 < circle.r:
		return True
	#d1 = math.hypot(circle.x-line.x1, circle.y-line.y1)
	#if d2 < circle.r: #true by construction
	#	return True
	r2 = (line.x2 - line.x1)**2 + (line.y2 - line.y1)**2
	u = ((circle.x - line.x1) * (line.x2 - line.x1) +
			(circle.y - line.y1) * (line.y2 - line.y1)) / r2 #distance from A to projection on line segment
	if u < 0 or u > 1:
		return False #outside line segment
	d = abs(((line.y1 - circle.y) * (line.x2 - line.x1) -
		(line.x1 - circle.x) * (line.y2 - line.y1)) / math.sqrt(r2)) #distance pt to line
	return (d <= circle.r)

# Same as the simple check, but do a quick check first so save time in most cases
def intersectsDistanceProjection(line, circle):
	# check if line to one side of box circumscribing the circle
	if (line.x1 > circle.xplusr and line.x2 > circle.xplusr) or \
	   (line.x1 < circle.xminusr and line.x2 < circle.xminusr) or \
	   (line.y1 > circle.yplusr and line.y2 > circle.yplusr) or \
	   (line.y1 < circle.yminusr and line.y2 < circle.yminusr):
		   return False
	# check if point 2 inside circle
	d2 = math.hypot(circle.x-line.x2, circle.y-line.y2)
	if d2 < circle.r:
		return True
	# check if point 1 inside circle
	#d1 = math.hypot(circle.x-line.x1, circle.y-line.y1)
	#if d1 < circle.r: #true by construction
	#	return True
	# check if min distance point is outside of the linesegment
	l2 = (line.x2 - line.x1)**2 + (line.y2 - line.y1)**2
	u = ((circle.x - line.x1) * (line.x2 - line.x1) +
			(circle.y - line.y1) * (line.y2 - line.y1)) / l2 #distance from A to projection on line segment
	if u < 0 or u > 1:
		return False
	# check circle-line distance
	d = abs(((line.y1 - circle.y) * (line.x2 - line.x1) -
		(line.x1 - circle.x) * (line.y2 - line.y1)) / math.sqrt(l2)) #distance pt to line
	return (d <= circle.r)

# A different collision checking algorithm attempt that ended up having worse performance
# attempting a different way to partition the space
def intersectsTangentProjection(line, circle):
	# check if line to one side of box circumscribing the circle
	if (line.x1 > circle.xplusr and line.x2 > circle.xplusr) or \
	   (line.x1 < circle.xminusr and line.x2 < circle.xminusr) or \
	   (line.y1 > circle.yplusr and line.y2 > circle.yplusr) or \
	   (line.y1 < circle.yminusr and line.y2 < circle.yminusr):
		   return False
	# check if point 2 inside circle
	d2 = math.hypot(circle.x-line.x2, circle.y-line.y2)
	if d2 < circle.r:
		return True
	# check if point 1 inside circle
	d1 = math.hypot(circle.x-line.x1, circle.y-line.y1)
	if d1 < circle.r:
		return True
	# check if line is shorter than distance to circle tangents
	# http://en.wikipedia.org/wiki/Thales%27_theorem
	l = math.hypot(line.x2 - line.x1, line.y2 - line.y1)
	if (l < d1 / 2 * 1.4142135623730951) or \
	   (l < d2 / 2 * 1.4142135623730951):
		return False
	# find equation of line of tangency
	# http://stackoverflow.com/questions/5683471/draw-a-line-from-a-point-to-opposite-tangents-on-a-circle-cone-wedge-shape-in-a
	c2x = (line.x1 + circle.x) / 2
	c2y = (line.y1 + circle.y) / 2
	d = d1 / 2
	a = circle.r2 / 2 / d

	p2x = circle.x + a * (c2x - circle.x) / d
	p2y = circle.y + a * (c2y - circle.y) / d
	h = math.sqrt(circle.r2 - a**2)
	x1 = p2x + h*(c2y - circle.y) / d
	y1 = p2y - h*(c2x - circle.x) / d
	x2 = p2x - h*(c2y - circle.y) / d
	y2 = p2y + h*(c2x - circle.x) / d
	
	# check if point in umbra
	jx = (circle.r2 - (y1-circle.y)*(line.y2-circle.y)) / (x1-circle.x) + circle.x
	jy = (circle.r2 - (x1-circle.x)*(line.x2-circle.x)) / (y1-circle.y) + circle.y
	kx = (circle.r2 - (y2-circle.y)*(line.y2-circle.y)) / (x2-circle.x) + circle.x
	ky = (circle.r2 - (x2-circle.x)*(line.y2-circle.x)) / (y2-circle.y) + circle.y

	# XXX: these tests do not correctly cover cases for inclusion in umbra
	if (jx < kx):
		if (line.x2 < jx) or (line.x2 > kx):
			return False
	else:
		if (line.x2 > jx) or (line.x2 < kx):
			return False
	
	if (jy < ky):
		if (line.y2 < jy) or (line.y2 > ky):
			return False
	else:
		if (line.y2 > jy) or (line.y2 < ky):
			return False

	return True

# for debugging, use both intersection methods to check consistency
def intersectsCombo(line, circle):
	i1 = intersectsDistanceProjection(line, circle)
	i2 = intersectsSimple(line,circle)
	if i1 != i2:
		print i1, i2
		ax = plt.axes()
		ax.axis('equal')
		ax.add_patch(patches.Circle((circle.x,circle.y),circle.r,color='blue'))
		ax.plot((line.x1,line.x2),(line.y1,line.x2),color='red')
		plt.show()
		assert i1 == i2
	return i1

intersects = intersectsDistanceProjection # the only good option

# Find the nearest neighbor by walking the tree
def nearestNeighborNaive((x,y), root):
	mindist = math.hypot(x-root.x, y-root.y) + root.cost
	bestnode = root
	node = root.child
	while node:	
		nodedist = math.hypot(x-node.x, y-node.y) + node.cost
		if INCLUDE_PATH_COST:
			nodedist += h(node, goal) # TOOD: this function could give the lower bound distance to the goal, etc
		if nodedist < mindist:
			mindist = nodedist
			bestnode = node

		if node.child:
			node = node.child
		elif node.sibling:
			node = node.sibling
		else:
			node = node.parent
			while node and not node.sibling:
				node = node.parent
			node = node.sibling if node else None
	return (mindist, bestnode)

# Find the nearest neighbor by delegating to the KDTree search
nearestNeighborKDTree = lambda query_point, root: root.kdtree.query(Point(query_point))

nearestNeighbor = nearestNeighborKDTree if NEAREST_NEIGHBOR_USES_KDTREE else nearestNeighborNaive

rt2xy = lambda r,t,x0,y0: (x0+r*math.cos(t), y0+r*math.sin(t)) # polar to cartesian helper for sampler

# Sampling function
def getSample(box, goal, samp_step):
	if OVERSAMPLE_GOAL and samp_step % OVERSAMPLE_GOAL == 0:
		return rt2xy(goal.r*math.sqrt(random.uniform(0,1)),random.uniform(0,2*math.pi), goal.x, goal.y) # uniform over goal circle
	return (random.uniform(box.x1, box.x2), random.uniform(box.y1, box.y2)) # uniform over box

####################
# Convenience storage types

class Point:
	def __init__(self, (x, y)):
		self.x = x
		self.y = y

class LocationNode:
	def __init__(self, (x, y), parent):
		self.x = x
		self.y = y
		self.parent = parent
		self.child = None
		self.cost = 0
		if parent:
			self.root = self.parent.root
			if NEAREST_NEIGHBOR_USES_KDTREE:
				self.root.kdtree.add(self)
			if INCLUDE_PATH_COST:
				self.cost = parent.cost + math.hypot(parent.x - x, parent.y - y)
			self.sibling = self.parent.child
			self.parent.child = self
		else:
			self.root = self
			if NEAREST_NEIGHBOR_USES_KDTREE:
				self.kdtree = KDTree(self)
			self.sibling = None

class Line:
	def __init__(self, (x1,y1), (x2,y2)):
		self.x1 = x1
		self.y1 = y1
		self.x2 = x2
		self.y2 = y2

class Obstacle:
	def __init__(self,x,y,r,robot_r=0):
		self.x = x
		self.y = y
		self.real_r = r
		r += robot_r
		self.r = r
		self.r2 = r**2
		self.xplusr = x+r
		self.xminusr = x-r
		self.yplusr = y+r
		self.yminusr = y-r

####################
# Initialization storage types for loading files

class Box:
	def __init__(self, fname):
		with file(fname) as f:
			m = re.match(r"^(-?[\d.]+)[\s,]*(-?[\d.]+)[\s,]*(-?[\d.]+)[\s,]*(-?[\d.]+)", f.readline())
			self.x, self.y, self.w, self.l = map(float,m.groups())
			self.x1 = self.x - self.l / 2
			self.x2 = self.x + self.l / 2
			self.y1 = self.y - self.w / 2
			self.y2 = self.y + self.w / 2

class Init:
	def __init__(self, fname):
		with file(fname) as f:
			m = re.match(r"^(-?[\d.]+)[\s,]*(-?[\d.]+)[\s,]*(-?[\d.]+)", f.readline())
			self.x, self.y, self.r = map(float,m.groups())

class Obstacles:
	def __init__(self, fname, robot_r):
		with file(fname) as f:
			self.list = []
			for line in f.readlines():
				if not line: continue
				m = re.match(r"^(-?[\d.]+)[\s,]*(-?[\d.]+)[\s,]*(-?[\d.]+)", line)
				self.list.append(Obstacle(*map(float,m.groups()),robot_r=robot_r))
	def __getitem__(self,x):
		return self.list[x]
				
class Goal:
	def __init__(self, fname):
		with file(fname) as f:
			m = re.match(r"^(-?[\d.]+)[\s,]*(-?[\d.]+)[\s,]*(-?[\d.]+)", f.readline())
			self.x, self.y, self.r = map(float,m.groups())
	def __contains__(self, point): # lets us write simply "node in goal" for solution test
		return numpy.hypot(point.x - self.x, point.y - self.y) < self.r if point != None else False
			

####################
# KDTree for node nearest neighbor calculation

""" KDTree implementation.
(modified heavily, Jameson Nash, 2011)
Matej Drame [matej.drame@gmail.com]
	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
	
	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
	
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""


class KDTreeNode():
	def __init__(self, lnode):
		self.lnode = lnode
		self.left = None
		self.right = None
	
class KDTreeNeighbours():
	""" Internal structure used in nearest-neighbours search.
	"""
	def __init__(self, query_point):
		self.query_point = query_point
		self.best_distance = 1e30
		self.current_best = None

	def add(self, lnode):
		sd = math.hypot(self.query_point.x-lnode.x, self.query_point.y-lnode.y)
		if not self.current_best or self.best_distance > sd:
			# replace the (previously) best node
			self.current_best = lnode
			self.best_distance = sd
	
class KDTree():
	""" KDTree implementation.
	"""
	
	def __init__(self, root):
		self.root_node = KDTreeNode(root)

	def add(self, lnode):
		node = self.root_node
		xaxis = False
		newnode = KDTreeNode(lnode)
		while True:
			# select axis based on depth so that axis cycles through all valid values
			xaxis = not xaxis
			if (xaxis and node.lnode.x > lnode.x) or \
			   (not xaxis and node.lnode.y > lnode.y):
				if not node.left:
					node.left = newnode
					return
				node = node.left
			else:
				if not node.right:
					node.right = newnode
					return
				node = node.right

	def _nn_search(self, node, query_point, depth, best_neighbours):
			if node == None:
				return
			
			# if we have reached a leaf, let's add to current best neighbours,
			# (if it's better than the worst onex)
			if node.left == None and node.right == None:
				best_neighbours.add(node.lnode)
				return
			
			# this node is no leaf
			
			# select dimension for comparison (based on current depth)
			xaxis = (depth % 2) == 0
			
			# figure out which subtree to search
			near_subtree = None # near subtree
			far_subtree = None # far subtree (perhaps we'll have to traverse it as well)
			
			# compare query_point and point of current node in selected dimension
			# and figure out which subtree is farther than the other
			if (xaxis and query_point.x < node.lnode.x) or \
			   (not xaxis and query_point.y < node.lnode.y):
				near_subtree = node.left
				far_subtree = node.right
			else:
				near_subtree = node.right
				far_subtree = node.left

			# recursively search through the tree until a leaf is found
			self._nn_search(near_subtree, query_point, depth+1, best_neighbours)

			# while unwinding the recursion, check if the current node
			# is closer to query point than the current best,
			# also, until t points have been found, search radius is infinity
			best_neighbours.add(node.lnode)
			
			# check whether there could be any points on the other side of the
			# splitting plane that are closer to the query point than the current best
			#print (node.lnode.x - query_point.x) if xaxis else (node.lnode.y - query_point.y), best_neighbours.best_distance
			if abs((node.lnode.x - query_point.x) if xaxis else (node.lnode.y - query_point.y)) < best_neighbours.best_distance:
				self._nn_search(far_subtree, query_point, depth+1, best_neighbours)
			return


	def query(self, query_point):
		# if there's no tree, there's no neighbors
		if self.root_node != None:
			neighbours = KDTreeNeighbours(query_point)
			self._nn_search(self.root_node, query_point, depth=0, best_neighbours=neighbours)
			result = (neighbours.best_distance,neighbours.current_best)
		else:
			result = None
		
		return result

####################

if __name__ == "__main__":
	main()

