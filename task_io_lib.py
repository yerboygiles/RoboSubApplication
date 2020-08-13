#!python3
# Author: Theodor Giles
# Created: 7/14/20
# Last Edited 7/27/20
# Description:
# This program manages the conversion of the mission.txt into commands that the MovementCommander can understand
# as well as the AI/TF/vision integration
#
import time
from movement_commander_v3 import MovementCommander


class TaskIO:
    # init
    def __init__(self, filename, usingvision, usingpixhawk):
        self.UsingVision = usingvision
        self.UsingPixhawk = usingpixhawk
        self.Filename = filename
        self.UsingVision = usingvision
        self.Active = False
        self.Movement = MovementCommander(self.UsingVision, self.UsingPixhawk)

    # get tasks from the .txt and completes them
    def get_tasks(self):
        # Testing
        Commands = open(self.Filename)

        for CommandLine in Commands:
            OuterCommandCounter = 0
            self.send_task(CommandLine)
        Commands.close()
        self.active = False

    # sends movement commands to movement commander
    def send_task(self, commandline):
        self.Movement.CommandControl(commandline)

    def terminate(self):
        self.Active = False
        self.Movement.Terminate()
