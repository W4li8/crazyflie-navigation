#!/usr/bin/env python3

# defines the attributes of a given node in the navigation graph
# valid state values are :
#                           - "unexplored"      | when no data is available for the drone
#                           - "clear"           | when it is estimated that the drone can pass through the block
#                           - "explored"        | when it is estimated that the drone can pass through the block
#                           - "blocked"         | when it is estimated that the drone cannot pass through the area
#                           - "unreachable"     | when it is estimated that the drone cannot pass through the area because all paths are blocked
#                           - "forbidden"       | when it is estiated that the drone should not pass through the area because it is near a blocked area
class NavNode():
    def __init__(self, x, y, state = "unexplored"):

        self.x = x      # x coordinate of the node in space
        self.y = y      # y coordinate of the node in space
        self.state = state
        self.pot = -1
        self.confidence = 0
    