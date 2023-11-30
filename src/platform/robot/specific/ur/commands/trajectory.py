#!/usr/bin/python3
import geometry_msgs.msg as geometry_msgs


class Trajectory:
    def __init__(self, name, kind, coord, move, tool_position):
        super().__init__()
        self.name = name
        self.kind = kind
        self.coord = coord
        self.move = move
        self.tool_position = tool_position

class Recette:
    def __init__(self, name):
        super().__init__()
        self.name = name
