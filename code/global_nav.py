#!/usr/bin/env python3

import networkx as nx                       # deals with the graph objects and all the graph-theoretic stuff
from nav_node import NavNode                # stores attributes for each node of the navigation graph
from copy import deepcopy
import sys
import math





class GlobalNav():

    # initializes the Global Nav Object
    def __init__(self, width, height, cols, rows, drone_x = 0, drone_y = 0,diag = False,decay = 0.025, radius = 2):
        
        ########################################################################
        #### INITIALIZING THE GRID 
        ########################################################################

        self.width = width      # actual width of the arena (in cm)
        self.height = height    # actual height of the arena (in cm)
        self.cols = cols        # width of the arena (in # of collumns)
        self.rows = rows        # height of the arena (in # of collumns)


        # Covering path planner parameter
        self.init_cover = False # false until the cover area is initialized
        self.c1 = 0             # lower c bound of the cover area
        self.c2 = 0             # upper c bound of the cover area
        self.r1 = 0             # lower r bound of the cover area
        self.r2 = 0             # upper r bound of the cover area
        self.cs = 0             # max pot val (used for display)
        self.coarse_tilings = dict() # coarse tiling datastructure (used for global minimum)
        self.levels = 0 # coarse tiling datastructure (used for global minimum)
        self.nav_state = "init"
        self.nav_wp = None

        self.radius = radius

        # confidence decay parameters
        self.decay = decay

        self.next_waypoint= None
        self.goal_waypoint = None

        self.path = None


        # Generating the networkx graph object that represents the space
        self.G = nx.Graph()
        self.pos = {}
        for c in range(self.cols):
            for r in range(self.rows):
                node = (c,r)
                coords = self.gridToSpace(c,r)
                self.G.add_node(node, obj=NavNode(coords[0],coords[1],'unexplored'))
                self.pos[node] = [coords[0],-coords[1]]
                if c>0 :
                    self.G.add_edge((c,r),(c-1,r))
                    self.G[(c,r)][(c-1,r)]['weight'] = 1
                    if r>0 : 
                        if diag:
                            self.G.add_edge((c,r),(c-1,r-1))
                            self.G[(c,r)][(c-1,r-1)]['weight'] = math.sqrt(2)
                            self.G.add_edge((c-1,r),(c,r-1))
                            self.G[(c-1,r)][(c,r-1)]['weight'] = math.sqrt(2)
                if r>0 :
                    self.G.add_edge((c,r),(c,r-1))
                    self.G[(c,r)][(c,r-1)]['weight'] = 1

        self.nG = deepcopy(self.G)
        ########################################################################
        #### INITIALIZING THE DRONE'S STATE
        ########################################################################

        self.drone_x = drone_x  # actual position x of the robot (in cm)
        self.drone_y = drone_y  # actual position y of the robot (in cm)
        self.trace = [(drone_x,drone_y)]    # trace of the drone coordinates

        drone_grid = self.spaceToGrid(self.drone_x,self.drone_y)

        self.dc = drone_grid[0] # grid position x of the drone
        self.dr = drone_grid[1] # grid position y of the drone

        self.visit(self.dc, self.dr)


        self.dtrace = [(self.dc, self.dr)]    # trace of the drone coordinates

        self.navPoints = []
        self.navCells = []


    # Converts grid coordinates to space coordinates
    def gridToSpace(self, col, row):
        sx = (self.width  / self.cols)  # spacing between nodes on the x axis
        sy = (self.height / self.rows)  # spacing between nodes on the y axis

        x = col * sx + sx / 2
        y = row * sy + sy / 2
        return [x, y]

    # Converts space coordinates to grid coordinates
    def spaceToGrid(self, x, y, delta = 2):

        sx = (self.width  / self.cols)  # spacing between nodes on the x axis
        sy = (self.height / self.rows)  # spacing between nodes on the y axis

        c = int(round(x/sx - 0.5))
        r = int(round(y/sy - 0.5))
        # if the rounding puts us in cell that exists and is not occupied then all good
        if self.nG.has_node((c,r)):
            return [c,r]
        # else choose the minimum distance cell satisfies the requirements
        else:
            minDist = sys.float_info.max
            mincoords = False
            for ci in range(c-delta,c+delta+1):
                for ri in range(r-delta,r+delta+1):
                    if ci != c or ri != r:
                        if self.nG.has_node((ci,ri)):
                            [cx,cy] = self.gridToSpace(ci,ri)
                            dist = math.sqrt( math.pow(x-cx,2)  + math.pow(y-cy,2) ) 
                            if dist < minDist:
                                minDist = dist
                                mincoords = [ci,ri]

            # if we couldnt map coordinates to a node at this point, call the panic function 
            if mincoords == False:
                mincoords = self.panic(x,y)
            return mincoords

    # called when we are out of the grid (think about it as a way to solve panic cases)
    def panic(self,x,y):
        minDist = sys.float_info.max
        dest = None

        # try to assign current node to closes clear node (L2 norm)
        for node in self.G.nodes:
            if self.G.nodes[node]['obj'].state == "unexplored" or self.G.nodes[node]['obj'].state == "clear"  or self.G.nodes[node]['obj'].state == "explored":
                nx = self.G.nodes[node]['obj'].x
                ny = self.G.nodes[node]['obj'].y
                dist = math.sqrt( math.pow(nx-x,2) + math.pow(ny-y,2) )
                if dist < minDist:
                    minDist = dist
                    dest = node
        
        # the only way dest is still unassigned at this point is that every single node on the map is blocked
        if dest == None:
            print("Map blocked, reseting map")
            #todo implement map reset here
            
        return dest



    # Converts space coordinates to grid coordinates but ensure the position returned is reachable (clear)
    def spaceToClear(self, x, y, delta = 2):

        sx = (self.width  / self.cols)  # spacing between nodes on the x axis
        sy = (self.height / self.rows)  # spacing between nodes on the y axis

        c = int(round(x/sx - 0.5))
        r = int(round(y/sy - 0.5))
        # if the rounding puts us in cell that exists and is not occupied then all good
        if self.nG.has_node((c,r)):
            return [c,r]
        # else choose the minimum distance cell satisfies the requirements
        else:
            minDist = sys.float_info.max
            mincoords = False
            for ci in range(max(c-delta,0),min(c+delta+1,self.cols)):
                for ri in range(max(r-delta,0),min(r+delta+1,self.rows)):
                    if ci != c or ri != r:
                        if self.G.nodes[(ci,ri)]['obj'].state == "clear" and self.nG.has_node((ci,ri)):
                            [cx,cy] = self.gridToSpace(ci,ri)
                            dist = math.sqrt( math.pow(x-cx,2)  + math.pow(y-cy,2) ) 
                            if dist < minDist:
                                minDist = dist
                                mincoords = [ci,ri]
            if mincoords == False:
                mincoords = self.spaceToGrid(x,y,10)
            return mincoords

    # Converts space coordinates to grid coordinates does it in a really stupid way (simply rounding to nearest int)
    def spaceToGridRound(self, x, y):

            sx = (self.width  / self.cols)  # spacing between nodes on the x axis
            sy = (self.height / self.rows)  # spacing between nodes on the y axis

            c = int(round(x/sx - 0.5))
            r = int(round(y/sy - 0.5))

            return [c,r]


    # marks a node as semantically "clear"
    def visit(self, c, r):
        node = (c,r)
        self.G.nodes[node]['obj'].state = "explored"
        self.G.nodes[node]['obj'].pot = 0 # "explored" cell have potential 0 for epsilon *
        self.G.nodes[node]['obj'].confidence = 1 # whenever we make a measurement, we update its confidence to 1

    def removeFromNavGraph(self,node):
        c = node[0]
        r = node[1]
        if self.nG.has_node(node):
            self.nG.remove_node(node)
        delEdges = [    ((c,r+1),(c+1,r)),
                        ((c+1,r),(c,r+1)),
                        ((c,r-1),(c+1,r)),
                        ((c+1,r),(c,r-1)),
                        ((c,r+1),(c-1,r)),
                        ((c-1,r),(c,r+1)),
                        ((c,r-1),(c-1,r)),
                        ((c-1,r),(c,r-1)) ]

        for ed in delEdges:
            if self.nG.has_edge(ed[0],ed[1]):
                self.nG.remove_edge(ed[0],ed[1])

        checkNodes = [  (c,r+1),
                        (c+1,r),
                        (c+1,r+1),
                        (c-1,r),
                        (c-1,r+1),
                        (c-1,r-1),
                        (c,r-1),
                        (c+1,r-1)]

    def sense_clear(self,c,r):
        if self.G.has_node((c,r)):
            if self.G.nodes[(c,r)]['obj'].state != "blocked" and self.G.nodes[(c,r)]['obj'].state != "explored" and self.G.nodes[(c,r)]['obj'].state != "forbidden" :
                self.G.nodes[(c,r)]['obj'].state = "clear"
                self.G.nodes[(c,r)]['obj'].confidence = 1

    # marks a node as semantically "blocked"
    def block(self, c, r):
        node = (c,r)
        if self.G.has_node(node) :
            self.G.nodes[node]['obj'].state = "blocked"     # set semantic state to blocked
            self.G.nodes[node]['obj'].pot = -1              # set potential to -1 (for epsilon*)

            self.G.nodes[node]['obj'].confidence = 1 # whenever we make a measurement, we update its confidence to 1
        adjNodes = []
        if self.radius == 1:
            adjNodes = [(c+1,r+1),(c+1,r),(c+1,r-1),(c,r+1),(c,r-1),(c-1,r+1),(c-1,r),(c-1,r-1)]
        if self.radius == 2:
            adjNodes = [(c+1,r+1),(c+1,r),(c+1,r-1),(c,r+1),(c,r-1),(c-1,r+1),(c-1,r),(c-1,r-1),
                        (c+2,r+1),(c+2,r),(c+2,r-1),(c-2,r+1),(c-2,r),(c-2,r-1),
                        (c+1,r+2),(c,r+2),(c-1,r+2),(c+1,r-2),(c,r-2),(c-1,r-2)]
        for adjNode in adjNodes:
            if self.G.has_node(adjNode) :
                if self.G.nodes[adjNode]['obj'].state != "blocked" and self.G.nodes[adjNode]['obj'].state != "unreachable" :    # set semantic state to blocked
                    self.G.nodes[adjNode]['obj'].state = "forbidden"     # set semantic state to forbidden
                    self.G.nodes[adjNode]['obj'].pot = -1              # set potential to -1 (for epsilon*)

                    self.G.nodes[adjNode]['obj'].confidence = 1 # whenever we make a measurement, we update its confidence to 1
                    self.removeFromNavGraph(adjNode)

        
        self.removeFromNavGraph(node)

        # check if we disconnected a part of the graph, if it is the case, set to unreachable areas inside of the disconnected graph
        # this means we maintain a navigation graph with a single connected component
        if nx.number_connected_components(self.nG) > 1:
#            print("deleting component")
            for component in list(nx.connected_components(self.nG)):
                if (self.dc,self.dr) not in component:
                    for node in component:
                        self.nG.remove_node(node)
                        self.G.nodes[node]['obj'].state = "unreachable" # set semantic state to unreachable
                        self.G.nodes[node]['obj'].pot = -1               # set potential to -1 (for epsilon*)

                        self.G.nodes[node]['obj'].confidence = 1 # whenever we make a measurement, we update its confidence to 1

    # marks a node as semantically "unexplored"
    def unexplore(self, c, r):
        node = (c,r)
        if self.G.has_node(node):
            self.G.nodes[node]['obj'].state = "unexplored"
            if self.nG.has_node(node) == False:
                self.nG.add_node(node)

            addEdges = [    ((c,r),(c,r+1)),
                            ((c,r),(c,r-1)),
                            ((c,r),(c+1,r+1)),
                            ((c,r),(c+1,r-1)),
                            ((c,r),(c+1,r)),
                            ((c,r),(c-1,r+1)),
                            ((c,r),(c-1,r-1)),
                            ((c,r),(c-1,r)) ]

            for ed in addEdges:
                if ed[1][0] > 0 and ed[1][1] > 0 and ed[1][0] < self.cols and ed[1][1] < self.rows :
                    if self.nG.has_edge(ed[0],ed[1]) == False:
                        self.nG.add_edge(ed[0],ed[1])

    # updates the drone position in grid coordinates
    def update(self, drone_x, drone_y,delta_t=1):
        self.drone_x = drone_x  # actual position x of the robot (in cm)
        self.drone_y = drone_y  # actual position y of the robot (in cm)

        drone_grid = self.spaceToGrid(self.drone_x,self.drone_y)

        self.dc = drone_grid[0] # grid position x of the drone
        self.dr = drone_grid[1] # grid position y of the drone
        self.trace.append((self.drone_x,self.drone_y))
        self.dtrace.append((self.dc,self.dr))

        self.visit(self.dc, self.dr)

        self.update_confidence(delta_t)

    # proposes a navigation waypoint to go to some destination point
    def navTo(self,dest_x,dest_y):
        self.navPoints.append((dest_x,dest_y))                      # keep history on destination waypoints
        [dest_c,dest_r] = self.spaceToGrid(dest_x,dest_y, delta=4)  # map the destination to a free cell on the grid (if occupied find near cell using spaceToGrid)
        self.navCells.append((dest_c,dest_r))                       # keep history on grid destination waypoints

        # if we are on an existing node, solve shortest path
        pos = (self.dc,self.dr)
        if self.nG.has_node(pos):                     
            # additional constraint : start path from a clear neighbor
            neighbors = self.nG.neighbors(pos)
            opt_len = sys.maxsize
            opt_path = [pos]
            for node in neighbors:
                if self.G.nodes[node]['obj'].state == "clear" or self.G.nodes[node]['obj'].state == "explored":
                    path = nx.shortest_path(self.nG, source=node, target=(dest_c,dest_r), method='dijkstra')
                    path.insert(0,pos)
                    if len(path) < opt_len:
                        opt_len = len(path)
                        opt_path = path 
            path = opt_path
            self.path = opt_path
            
        # if we are not on an existing node, something is wrong, go to a clear node nearby
        else:
            [self.dc,self.dr] = self.spaceToClear(self.drone_x,self.drone_y,delta = 3)
            dest = self.gridToSpace(self.dc,self.dr)
            return [dest[0],dest[1]]
        if len(path)> 1:
            nwx = path[1][0]
            nwy = path[1][1]
        else :
            nwx = path[0][0]
            nwy = path[0][1]
        [npx, npy] = self.gridToSpace(nwx, nwy)
        return [npx, npy]

    # initializes an epsilon* potential field over a given area bounded by c1 <= c <= c2 and r1 <= r <= r2
    def cover_area(self,c1,r1,c2,r2):
        self.c1 = c1
        self.c2 = c2
        self.r1 = r1
        self.r2 = r2
        # reset the MAPS
        self.nav_state = "nav_cover"
        self.nav_wp = self.gridToSpace(c1,r2)
        self.coarse_tilings = dict() # coarse tiling datastructure (used for global minimum)
        self.levels = 0 # coarse tiling datastructure (used for global minimum)
        for node in self.G.nodes:
            self.G.nodes[node]['obj'].pot = -1

        # start by defining a potential for every cell in the area
        self.cs = self.r2 - self.r1 + 1
        for c in range(c1,c2+1):
            for r in range(r1,r2+1):
                node = (c,r)
                if self.G.has_node(node)  :
                    if self.G.nodes[node]['obj'].state == "blocked" or self.G.nodes[node]['obj'].state == "unreachable"  or self.G.nodes[node]['obj'].state == "forbidden" :
                        self.G.nodes[node]['obj'].pot = -1
                    else:
                        self.G.nodes[node]['obj'].pot = r - r1 + 2

        # defining the coarser tilings : 

        self.recursive_area_splitting(c1,r1,c2,r2)

    def is_in_cover_area(self,c,r):
        if c >= self.c1 and c <= self.c2:
            if r >= self.r1 and r <= self.r2:
                return True
        return False

    def recursive_area_splitting(self,c1,r1,c2,r2):
        level_dict = dict()

        nc = c2-c1
        rc = r2-r1
        L0 = math.ceil(max(nc,rc)/2)
        # iterate over all levels
        self.levels = math.ceil(math.log(L0,2))
        for level in range(1,self.levels + 1):
            levelGraph = nx.Graph()
            
            #iterate over all nodes and assign them to a supernode
            for c in range(math.ceil(nc/math.pow(2,level))):

                for r in range(math.ceil(rc/math.pow(2,level))):

                    levelGraph.add_node((c,r))
                    if(c > 0):
                        levelGraph.add_edge((c,r),(c-1,r))
                    if(r > 0):
                        levelGraph.add_edge((c,r),(c,r-1))
                        if(c > 0):
                            levelGraph.add_edge((c,r-1),(c-1,r))
                            levelGraph.add_edge((c,r),(c-1,r-1))
                    levelGraph.nodes[(c,r)]["subnodes"] = []
                    for delta_c in range(int(math.pow(2,level))):
                        for delta_r in range(int(math.pow(2,level))):
                            
                            subnode_c = c1 + c * math.pow(2,level) + delta_c
                            subnode_r = r1 + r * math.pow(2,level) + delta_r
                            if subnode_c < c2 and subnode_r < r2:
                                levelGraph.nodes[(c,r)]["subnodes"].append((int(subnode_c),int(subnode_r)))


            level_dict[level] = levelGraph
            
        self.coarse_tilings =  level_dict

    # returns the id of the supernode on a given level that contains the node "node"
    def super_node_id(self,node, level):
        

        #basically check all nodes, no clever datastructure tricks
        for supernode in self.coarse_tilings[level].nodes:
            for subnode in self.coarse_tilings[level].nodes[supernode]["subnodes"]:
                if subnode == node:
                    return supernode
        
        # print("No supernode found for node : " + str(node))



    # returns the potential value of a coarse tiling node (supoernode) Computation as described in Song & Gupta's paper for epsilon*
    def potential_of_supernode(self,supernode,level):
        pot_acc = 0
        pot_d = 0
        for subnode in self.coarse_tilings[level].nodes[supernode]["subnodes"]:
            if self.G.has_node(subnode):
                if self.G.nodes[subnode]['obj'].pot > 0:    
                    pot_d = pot_d + 1
                    pot_acc = pot_acc + self.G.nodes[subnode]['obj'].pot
        if pot_acc == 0:
            return 0
        else :
            return pot_acc/pot_d

    def pick_min_subnode_in_supernode(self,supernode,level):
        minVal = sys.maxsize
        minNode = None

        for subnode in self.coarse_tilings[level].nodes[supernode]["subnodes"]:
            if self.G.nodes[subnode]['obj'].pot > 0:
                if self.G.nodes[subnode]['obj'].pot < minVal:
                    minVal = self.G.nodes[subnode]['obj'].pot
                    minNode = subnode
        return minNode
            

    # computes the next waypoint using epsilon*
    def cover(self):

        # if we are out of the cover area, navigate back into it
        # if self.is_in_cover_area(self.dc,self.dr) == False and self.nav_state != "nav_cover":
        #     print( "out of area, coordinates are" +str((self.dc,self.dr))  )
        #     pot = 0 
        #     dest = None
        #     for supernode in self.coarse_tilings[self.levels].nodes:

        #         wp = self.pick_min_subnode_in_supernode(supernode,self.levels)
        #         if self.G.has_node(wp) and self.is_in_cover_area(wp[0],wp[1]):
        #             if self.G.nodes[(wp[0],wp[1])]['obj'].pot > pot:
        #                 pot = self.G.nodes[(wp[0],wp[1])]['obj'].pot
        #                 dest = wp
        #     if dest == None:

        #         return (self.dc,self.dr)
        #         # and return something to signal the FSM

        #     wp0 = self.gridToSpace(dest[0],dest[1])
        #     self.nav_state = "nav_cover"
        #     self.nav_wp = (dest[0],dest[1])
        #     return self.navTo(wp0[0],wp0[1])


        # handles waypoint navigation
        if self.nav_state =="nav_cover":
            grid_node = self.spaceToGrid(self.nav_wp[0],self.nav_wp[1])
            node = (grid_node[0],grid_node[1])
            if node == (self.dc,self.dr):
                self.nav_state = "cover"
            elif self.G.nodes[node]['obj'].state == "blocked" or self.G.nodes[node]['obj'].state == "unreachable"  or self.G.nodes[node]['obj'].state == "forbidden" :
                self.nav_state = "cover"
            else:
                return self.navTo(self.nav_wp[0],self.nav_wp[1])

        pos = (self.dc,self.dr)
        maxPot = 0
        wp = pos
        for node in self.G.neighbors(pos):
            if self.G.nodes[node]['obj'].pot > maxPot:
                maxPot = self.G.nodes[node]['obj'].pot
                wp = node
                self.nav_state = "cover"
        
        #detect local minimums:
        if wp == pos:
            #if detected : gradually go up coarse tiling levels util either something is found or space is covered
            for level in range(1,self.levels+1):

                maxPot = 0
                superpos = self.super_node_id(pos,level) # find super node containing our position on that level, evaluate the potential of our neighbors
                if self.coarse_tilings[level].has_node(superpos):
                    neighbors = self.coarse_tilings[level].neighbors(superpos)

                    for neighbor in neighbors :
                        if self.potential_of_supernode(neighbor,level) > maxPot:
                            maxPot = self.potential_of_supernode(neighbor,level)
                            swp = neighbor
                    if maxPot > 0: # if we are not in a local minimum at that level, navigate to neighbor
                        wp = self.pick_min_subnode_in_supernode(swp,level)
                        wp0 = self.gridToSpace(wp[0],wp[1])
                        self.nav_state = "nav_cover"
                        self.nav_wp = (wp0[0],wp0[1])
                        return self.navTo(wp0[0],wp0[1])

        return self.gridToSpace(wp[0],wp[1])

    def update_conf(self,node,delta_t):
        self.G.nodes[node]['obj'].confidence = (1-self.decay) * self.G.nodes[node]['obj'].confidence
        if self.G.nodes[node]['obj'].confidence < 0.1:
            self.unexplore(node[0],node[1])

    def update_confidence(self,delta_t):
        for node in self.G.nodes:
            self.update_conf(node,delta_t)

    #Todo
    def sense(self,pos,sensor):

        sx = (self.width  / self.cols)

        drone_x = pos[0]*100
        drone_y = pos[1]*100
        yaw = (pos[-1]/180)*math.pi
        sensor_angle = [0, math.pi, math.pi/2, -math.pi/2]

        period = sx/3



        for i in range(4):
            dist = 0
            uvx = math.cos(sensor_angle[i]+yaw)
            uvy = math.sin(sensor_angle[i]+yaw)
            while dist < 80 and dist < sensor[i]*100:
                dist = dist + sx
                sense_x = drone_x + uvx * dist
                sense_y = drone_y + uvy * dist
                [c,r] = self.spaceToGridRound(sense_x,sense_y)
                self.sense_clear(c,r)
            if dist >= sensor[i]*100:
                [c,r] = self.spaceToGridRound(sense_x,sense_y)
                self.block(c,r)
    #wrapper function for the entire update step 
    def get_waypoint(self,pos,sensor,delta_t):
        measured_x = pos[0] * 100
        measured_y = pos[1] * 100
        self.update(measured_x,measured_y,delta_t)
        self.sense(pos,sensor)
        # return self.cover()
        if self.goal_waypoint != None:
            goal = self.gridToSpace(self.goal_waypoint[0],self.goal_waypoint[1])
        else:
            goal = (self.drone_x,self.drone_y)
        return self.navTo(goal[0],goal[1])

