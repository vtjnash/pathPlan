####################
# Configuration Parameters
type PathPlanConfig
    DEBUG::Int
    p::Float64
    INCLUDE_PATH_COST::Bool
    OVERSAMPLE_GOAL::Int
    ENVIRONMENT::String
    REPEAT::Int
    NEAREST_NEIGHBOR_USES_KDTREE::Bool
    PathPlanConfig() = new(0,1,false,false,"sample_environment",0,false)
end
const config = PathPlanConfig()
config.DEBUG = 0 # enable debug view every X steps
config.p = .2 # move p% of the distance each step, 0 < p <= 1
config.INCLUDE_PATH_COST = false # compute path costs, conflicts with KDTree option, do not use
config.OVERSAMPLE_GOAL = 32 # sample in goal every X steps
config.ENVIRONMENT = "giant" # environment name to run, must match folder in current directory
config.REPEAT = 10 # number of times to average over
config.NEAREST_NEIGHBOR_USES_KDTREE = true # using KDTree to find nearest neighbor
#random.seed(2678900629721256704L) # uncomment to make simulations repeatable

checkCollisions(x...) = checkCollisionsNaive(x...)
intersects(x...) = intersectsDistanceProjection(x...) #OR intersectsNaive
nearestNeighbor(x...) = nearestNeighborKDTree(x...) #OR nearestNeighborNaive
h(node,goal) = 0  #TODO: this function could give the lower bound distance to the goal, etc

push(LOAD_PATH, "../gaston")

####################
# Execution main blocks
require("gaston.jl")

function main()
    println("Environment ", config.ENVIRONMENT)
    cd(config.ENVIRONMENT)
    if true
        box = Box("box.txt")
        if config.DEBUG > 0 println("box $(box.x) $(box.y) $(box.l) $(box.w) $(box.x1) $(box.x2) $(box.y1) $(box.y2)") end
        init = Init("init.txt")
        obstacles = Obstacles("obstacles.txt",init.r)
        goal = Goal("goal.txt")
        if config.DEBUG > 0 plotEnviron(box,init,obstacles,goal,nothing,nothing) end
        totaltime = 0
        tree = node = nothing
        for i = 0:config.REPEAT
            tree = LocationNode((init.x, init.y), nothing)
            t1 = time()
            node = pathSearch(box,init,obstacles,goal,tree)
            t2 = time() - t1
            println("Took: $(t2*1e3)", " ms")
            if i > 0 totaltime += t2 end
        end
        if config.REPEAT > 0 println("Average time of $(config.REPEAT) runs: $(totaltime/config.REPEAT*1e3) ms") end
        plotEnviron(box,init,obstacles,goal,tree,node)
        return (box, init, obstacles, goal, tree, node)
    end
end

function plotEnviron(box,init,obstacles,goal,tree,node)
    if CURRENT_OS == :OSX
        set_terminal("aqua")
    end
    ax = figure()
    c = CurveConf()
    c.plotstyle = "lines"
    c.color = "black"
    addcoords([box.x1,box.x1,box.x2,box.x2,box.x1],[box.y1,box.y2,box.y2,box.y1,box.y1],c)
    addconf(AxesConf())
    gnuplot_send("set size ratio -1")
    gnuplot_send("set style fill solid 1.0 border -1")
    gnuplot_send("set object circle at first $(goal.x),$(goal.y) size $(goal.r) fc rgb \"blue\"")
    gnuplot_send("set object circle at first $(init.x),$(init.y) size $(init.r) fc rgb \"green\"")
    for ob = obstacles.list
        gnuplot_send("set object circle at first $(ob.x),$(ob.y) size $(ob.real_r) fc rgb \"black\"")
    end
    if tree != nothing && tree.child != nothing
        plotTree(ax, tree.child)
    end
    if node != nothing
        plotReverseTree(ax, node)
        gnuplot_send("set object circle at first $(node.x),$(node.y) size $(init.r) fc rgb \"green\"")
    end
    llplot()
    sleep(0.1)
end

function plotTree(ax,location)
	parent = location.parent
    c = CurveConf()
    c.plotstyle = "lines"
    c.color = "red"
    addcoords([parent.x,location.x],[parent.y,location.y],c)
    if location.child != nothing; plotTree(ax, location.child) end
    if location.sibling != nothing; plotTree(ax, location.sibling) end
end

function plotReverseTree(ax, location)
    while location.parent != nothing
	    parent = location.parent
        c = CurveConf()
        c.plotstyle = "lines"
        c.color = "green"
        addcoords([parent.x,location.x],[parent.y,location.y],c)
        location = parent
    end
end

####################
# Primary algorithms

# RRT Search
function pathSearch(box,init,obstacles,goal,tree)
    node = nothing
    samp_step = 0
    while !in(node,goal)
        samp_step += 1
        node = addNode(obstacles, tree, getSample(box, goal, samp_step))
        if config.DEBUG > 0 && (samp_step % config.DEBUG == 0) plotEnviron(box,init,obstacles,goal,tree,node) end
    end
    if config.DEBUG > 0 println("Sample Steps: $samp_step") end
    return node
end

# Add node based on "sample", if feasible
function addNode(obstacles, tree, sample)
    dist, nearest = nearestNeighbor(sample, tree)
    dnew = (nearest.x*(1-config.p) + sample[1]*config.p,
        nearest.y*(1-config.p)+sample[2]*config.p)
    if checkCollisions(Line((nearest.x, nearest.y), dnew), obstacles)
        return LocationNode(dnew, nothing)
    else
        return LocationNode(dnew, nearest)
    end
end

function checkCollisionsNaive(line, obstacles)
    for obstacle in obstacles.list
        if intersects(line, obstacle)
            return true
        end
    end
    return false
end

function intersectsSimple(line, circle)
    d2 = hypot(circle.x-line.x2, circle.y-line.y2)
    if d2 < circle.r
        return true
    end
    #d1 = hypot(circle.x-line.x1, circle.y-line.y1)
    #if d2 < circle.r #false by construction
    #   return true
    #end
    r2 = (line.x2 - line.x1)^2 + (line.y2 - line.y1)^2
    u = ((circle.x - line.x1) * (line.x2 - line.x1) +
        (circle.y - line.y1) * (line.y2 - line.y1)) / r2 #distance from A to projection on line segment
    if u < 0 || u > 1
        return false #outside line segment
    end
    d = abs(((line.y1 - circle.y) * (line.x2 - line.x1) -
        (line.x1 - circle.x) * (line.y2 - line.y1)) / sqrt(r2)) #distance pt to line
    return (d <= circle.r)
end

# Same as the simple check, but do a quick check first so save time in most cases
function intersectsDistanceProjection(line,circle)
    # check if line to one side of box circumscribing the circle
    if (line.x1 > circle.xplusr && line.x2 > circle.xplusr) ||
       (line.x1 < circle.xminusr && line.x2 < circle.xminusr) ||
       (line.y1 > circle.yplusr && line.y2 > circle.yplusr) ||
       (line.y1 < circle.yminusr && line.y2 < circle.yminusr)
        return false
    end
    # check if point 2 inside circle
    d2 = hypot(circle.x-line.x2, circle.y-line.y2)
    if d2 < circle.r
        return true
    end
    # check if point 1 inside circle
    #d1 = hypot(circle.x-line.x1, circle.y-line.y1)
    #if d2 < circle.r #false by construction
    #   return true
    #end
    # check if min distance pint is outside of the linesegment
    l2 = (line.x2 - line.x1)^2 + (line.y2 - line.y1)^2
    u = ((circle.x - line.x1) * (line.x2 - line.x1) +
        (circle.y - line.y1) * (line.y2 - line.y1)) / l2 #distance from A to projection on line segment
    if u < 0 || u > 1
        return false #outside line segment
    end
    # check circle-line distance
    d = abs(((line.y1 - circle.y) * (line.x2 - line.x1) -
        (line.x1 - circle.x) * (line.y2 - line.y1)) / sqrt(l2)) #distance pt to line
    return (d <= circle.r)
end

# for debugging, use both intersection methods to check consistency
function intersectsCombo(line, circle)
	i1 = intersectsDistanceProjection(line, circle)
	i2 = intersectsSimple(line,circle)
	if i1 != i2
        ax = figure()
		println("$i1 $i2 $line $circle")
        gnuplot_send("set size ratio -1")
        gnuplot_send("set style fill solid 1.0 border -1")
        gnuplot_send("set object circle at first $(circle.x),$(circle.y) size $(circle.r) fc rgb \"black\"")
        c = CurveConf()
        c.plotstyle = "lines"
        c.color = "red"
        addcoords([line.x1,line.x2],[line.y1,line.y2],c)
        addconf(AxesConf())
        llplot()
    end
    @assert i1 == i2
	return i1
end

function nearestNeighborNaive(xy, root)
    x, y = xy
    mindist = hypot(x-root.x, y-root.y) + root.cost
    bestnode = root
    node = root.child
    while node != nothing
        nodedist = hypot(x-node.x, y-node.y) + node.cost
        if config.INCLUDE_PATH_COST
            nodedist += h(node, goal)
        end
        if nodedist < mindist
            mindist = nodedist
            bestnode = node
        end

        if node.child != nothing
            node = node.child
        elseif node.sibling != nothing
            node = node.sibling
        else
            node = node.parent
            while node != nothing && node.sibling == nothing
                node = node.parent
            end
            node = (if node node.sibling else nothing end)
        end
    end
    return (mindist, bestnode)
end

nearestNeighborKDTree(query_point, root) = query(root.kdtree, Point(query_point))

rt2xy(r,t,x0,y0) = (x0+r*cos(t), y0+r*sin(t)) # polar to cartesian helper for sampler

# Sampling function
randrange(bounds) = rand()*(bounds[2]-bounds[1])+bounds[1]
function getSample(box, goal, samp_step)
    if config.OVERSAMPLE_GOAL > 0 && samp_step % config.OVERSAMPLE_GOAL == 0
        return rt2xy(goal.r*sqrt(randrange((0,1))), randrange((0,2*pi)), goal.x, goal.y) # uniform over goal circle
    else
        return (randrange((box.x1,box.x2)), randrange((box.y1,box.y2))) # uniform over box
    end
end

####################
# Convenience storage types

type Point
    x::Float64
    y::Float64
    Point(xy) = new(xy[1],xy[2])
end
type Line
    x1::Float64
    y1::Float64
    x2::Float64
    y2::Float64
    Line(xy1, xy2) = new(xy1[1],xy1[2],xy2[1],xy2[2])
end
type Obstacle
    x::Float64
    y::Float64
    real_r::Float64
    r::Float64
    r2::Float64
    xplusr::Float64
    xminusr::Float64
    yplusr::Float64
    yminusr::Float64
    Obstacle(x,y,r,robot_r) = (real_r=r; r+=robot_r; new(x,y,real_r,r,r^2,x+r,x-r,y+r,y-r))
end
type LocationNode
    x::Float64
    y::Float64
    parent::Union(LocationNode,Nothing)
    child::Union(LocationNode,Nothing)
    sibling::Union(LocationNode,Nothing)
    cost::Float64
    root::LocationNode
    kdtree
    function LocationNode(xy, parent)
        x,y=xy
        self = new(x,y,parent,nothing,nothing,0)
        if parent != nothing
            self.root = parent.root
            if config.NEAREST_NEIGHBOR_USES_KDTREE
                add(self.root.kdtree, self)
            end
            if config.INCLUDE_PATH_COST
                self.cost = parent.cost + hypot(parent.x - x, parent.y - y)
            end
            self.sibling = parent.child
            parent.child = self
        else
            self.root = self
            if config.NEAREST_NEIGHBOR_USES_KDTREE
                self.kdtree = KDTree(self)
            end
        end
        self
    end
end
show(io,lnode::LocationNode) = print("LocationNode(x=$(lnode.x),y=$(lnode.y))")

####################
# Initialization storage types for loading files
type Box
    x::Float64
    y::Float64
    w::Float64
    l::Float64
    x1::Float64
    x2::Float64
    y1::Float64
    y2::Float64
end
function Box(fname::String)
    f = open(fname)
    m = match(r"^(-?[\d.]+)[\s,]*(-?[\d.]+)[\s,]*(-?[\d.]+)[\s,]*(-?[\d.]+)", readline(f))
    x,y,w,l = map(float, m.captures)
    Box(x,y,w,l,
        x-l/2, x+l/2,
        y-w/2, y+w/2)
end

type Init
    x::Float64
    y::Float64
    r::Float64
end
function Init(fname::String)
    f = open(fname)
    m = match(r"^(-?[\d.]+)[\s,]*(-?[\d.]+)[\s,]*(-?[\d.]+)", readline(f))
    x,y,r = map(float, m.captures)
    Init(x,y,r)
end

type Obstacles
    list::Vector{Obstacle}
end
function Obstacles(fname::String, robot_r::Float64)
    f = open(fname)
    list = Obstacle[]
    while !eof(f)
        line = readline(f)
        if "" == line continue end
        m = match(r"^(-?[\d.]+)[\s,]*(-?[\d.]+)[\s,]*(-?[\d.]+)", line)
        x,y,r = map(float, m.captures)
        push(list, Obstacle(x,y,r,robot_r))
    end
    Obstacles(list)
end
ref(self::Obstacles, x) = self.list[x]

type Goal
    x::Float64
    y::Float64
    r::Float64
end
function Goal(fname::String)
    f = open(fname)
    m = match(r"^(-?[\d.]+)[\s,]*(-?[\d.]+)[\s,]*(-?[\d.]+)", readline(f))
    x,y,r = map(float, m.captures)
    Goal(x,y,r)
end
in(point, self::Goal) = if point != nothing hypot(point.x - self.x, point.y - self.y) < self.r else false end
in(::Nothing, self::Goal) = false

####################
# KDTree for node nearest neighbor calculation

#KDTree implementation.
#(modified heavily, Jameson Nash, 2011)
#Matej Drame [matej.drame@gmail.com]
#	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
#	
#	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
type KDTreeNode
    lnode::LocationNode
    left::Union(KDTreeNode,Nothing)
    right::Union(KDTreeNode,Nothing)
    KDTreeNode(lnode::LocationNode) = new(lnode, nothing, nothing)
end

# Internal structure used in nearest-neighbours search.
type KDTreeNeighbours
    query_point::Point
    best_distance::Float64
    current_best::Union(LocationNode,Nothing)
    KDTreeNeighbours(query_point::Point) = new(query_point, Inf, nothing)
end
function add(self::KDTreeNeighbours, lnode::LocationNode)
    sd = hypot(self.query_point.x - lnode.x, self.query_point.y - lnode.y)
    if self.current_best == nothing || self.best_distance > sd
        # replace the (previously) best node
        self.current_best = lnode
        self.best_distance = sd
    end
end

type KDTree
    root_node::KDTreeNode
    KDTree(lnode::LocationNode) = new(KDTreeNode(lnode))
end
function add(self::KDTree, lnode::LocationNode)
    node = self.root_node
    xaxis = false
    newnode = KDTreeNode(lnode)
    while true
        xaxis = !xaxis
        if (xaxis && node.lnode.x > lnode.x) ||
            (!xaxis && node.lnode.y > lnode.y)
            if node.left == nothing
                node.left = newnode
                return
            end
            node = node.left
        else
            if node.right == nothing
                node.right = newnode
                return
            end
            node = node.right
        end
    end
end
function _nn_search(self::KDTree, node, query_point, depth, best_neighbours)
    if node == nothing
        return
    else
			# if we have reached a leaf, let's add to current best neighbours,
			# (if it's better than the worst onex)
			if node.left == nothing && node.right == nothing
				add(best_neighbours, node.lnode)
				return
            end
			# this node is no leaf
			
			# select dimension for comparison (based on current depth)
			xaxis = (depth % 2) == 0
			
			# figure out which subtree to search
			near_subtree = nothing # near subtree
			far_subtree = nothing # far subtree (perhaps we'll have to traverse it as well)
			
			# compare query_point and point of current node in selected dimension
			# and figure out which subtree is farther than the other
			if (xaxis && query_point.x < node.lnode.x) ||
			   (!xaxis && query_point.y < node.lnode.y)
				near_subtree = node.left
				far_subtree = node.right
			else
                near_subtree = node.right
				far_subtree = node.left
            end

			# recursively search through the tree until a leaf is found
			_nn_search(self, near_subtree, query_point, depth+1, best_neighbours)

			# while unwinding the recursion, check if the current node
			# is closer to query point than the current best,
			# also, until t points have been found, search radius is infinity
			add(best_neighbours, node.lnode)
			
			# check whether there could be any points on the other side of the
			# splitting plane that are closer to the query point than the current best
			#print (node.lnode.x - query_point.x) if xaxis else (node.lnode.y - query_point.y), best_neighbours.best_distance
			dist = abs(if xaxis node.lnode.x - query_point.x else node.lnode.y - query_point.y end)
            if dist < best_neighbours.best_distance
                _nn_search(self, far_subtree, query_point, depth+1, best_neighbours)
            end
			return
    end
end
function query(self::KDTree, query_point)
		# if there's no tree, there's no neighbors
		if self.root_node != nothing
			neighbours = KDTreeNeighbours(query_point)
			_nn_search(self, self.root_node, query_point, 0, neighbours)
			result = (neighbours.best_distance,neighbours.current_best)
		else
			result = nothing
        end
		
		return result
end

####################

if !isinteractive()
    main()
end

