#!/usr/bin/python3


class Recipe:
    def __init__(self, name, trajectories, devices):
        super().__init__()
        self.name = name
        self.trajectories = trajectories
        self.devices = devices
