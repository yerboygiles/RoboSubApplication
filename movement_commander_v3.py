#!python3
# Author: Theodor Giles
# Created: 7/14/20
# Last Edited 8/6/20
# Description:
# This program manages the commands/movement/physical control of the RoboSub V1
#
import time
import pyfirmata
import math

# setting up board serial port
print("Communicating with Arduino...")
# something so the serial buffer doesn't overflow
board = pyfirmata.ArduinoMega('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_55839313438351417131-if00')
iter8r = pyfirmata.util.Iterator(board)
iter8r.start()

print("Communication with Arduino started...")


# ROBOSUB


class MovementCommander:

    # initialize everything to supposed starting position
    def __init__(self, usingvision, usingpixhawk):
        self.UsingVision = usingvision
        self.UsingPixHawk = usingpixhawk
        if self.UsingVision:
            from ai_lib import AI
            self.VisionAI = AI("TensorFlow_Graph/Tflite")
            print("MovementCommander is using Vision AI...")
        else:
            print("MovementCommander is not using Vision AI...")

        if self.UsingPixHawk:
            from pixhawk_data import PixHawk
            self.PixHawk = PixHawk()
            print("MovementCommander is using PixHawk...")
        else:
            print("MovementCommander is not using PixHawk...")

        # possible variable I'll use later for determining how "off path/kilter" we are
        # from a designated target/orientation. basically if it's too high of a #,
        # sirens go off and everybody freaks out and the robot tries extra hard to get
        # back on route
        self.OffKilter = 0

        # normal x,y,z orientation stuff, gonna set to sensor values once I get them
        self.Pitch = 0
        self.Roll = 0
        self.Yaw = 0

        self.WantedPitch = 0
        self.Roll = 0
        self.Yaw = 0

        # thruster hardpoint classes
        self.ThrusterLB = ThrusterDriver(2)  # left back
        self.ThrusterLF = ThrusterDriver(4)  # left front
        self.ThrusterRB = ThrusterDriver(3)  # right back
        self.ThrusterRF = ThrusterDriver(5)  # right front
        self.ThrusterBL = ThrusterDriver(6)  # back left
        self.ThrusterBR = ThrusterDriver(7)  # back right
        self.ThrusterFL = ThrusterDriver(8)  # front left
        self.ThrusterFR = ThrusterDriver(9)  # front right
        print("Wait 3 to arm thrusters...")
        time.sleep(3)

        # power values to set to the thruster hardpoints
        # horizontally orientated
        self.PowerLB = 0
        self.PowerLF = 0
        self.PowerRB = 0
        self.PowerRF = 0
        # vertically orientated
        self.PowerBL = 0
        self.PowerBR = 0
        self.PowerFR = 0
        self.PowerFL = 0

        # initialize thruster values to brake (self.PowerXX set to 0^)
        self.ThrusterLB.SetSpeed(self.PowerLB)
        self.ThrusterLF.SetSpeed(self.PowerLF)
        self.ThrusterRB.SetSpeed(self.PowerRB)
        self.ThrusterRF.SetSpeed(self.PowerRF)
        self.ThrusterBL.SetSpeed(self.PowerBL)
        self.ThrusterBR.SetSpeed(self.PowerBR)
        self.ThrusterFL.SetSpeed(self.PowerFL)
        self.ThrusterFR.SetSpeed(self.PowerFR)

        # string list of movement commands, because I thought I'd make
        # the index number of each command streamlined with other
        # functions, but it seems a bit detrimental the more I work with
        # it.

        # basic: these commands are just normal up, down, turn, etc.
        self.BASIC_MOVEMENT_COMMANDS = [
            "FORWARD",
            "REVERSE",
            "CLOCKWISE TURN",
            "COUNTERCLOCKWISE TURN",
            "DIVE",
            "SURFACE",
            "IDLE"]

        # advanced: these commands are much more complicated, will need to
        # develop pathing and a lot of vision/gyro/position integration
        self.ADVANCED_MOVEMENT_COMMANDS = [
            "LOG START POINT",
            "RETURN TO START",
            "MOVE TO TARGET",
            "RAM TARGET",
            "TURN TO TARGET",
            "TURN TO ANGLE"]
        # currently only for firing torpedoes, maybe a claw action later on?
        self.SUPPLEMENTARY_COMMANDS = [
            "FIRE TORPEDO"
        ]
        # name of object to target sent to TF/openCV AI
        self.TO_TARGET = ""

        # possible targets, matches up with labelmap.txt to make easier
        self.POSSIBLE_TARGETS = [
            "red_buoy",
            "blue_buoy",
            "green_buoy",
            "orange_buoy",
            "gate"]
        self.TargetList = []
        print("MovementCommander initialized...")

    # checking to see if the sent string is actually a command
    def IsStringACommand(self, searchtarg):
        Command = "NONE"
        Found = False
        i = 0
        if not Found:
            for command in self.POSSIBLE_TARGETS:
                if command == searchtarg:
                    Command = "TARGET"
                    Found = True
                if not Found:
                    i += 1
        if not Found:
            i = 0
            for command in self.BASIC_MOVEMENT_COMMANDS:
                if command == searchtarg:
                    Command = "BASIC"
                    Found = True
                if not Found:
                    i += 1
        if not Found:
            i = 0
            for command in self.ADVANCED_MOVEMENT_COMMANDS:
                if command == searchtarg:
                    Command = "ADVANCED"
                    Found = True
                if not Found:
                    i += 1
        return Command, i

    # Concept code, basically for checking if the Sub has already seen the detected object.
    def IsTargetInMemory(self, label, x, y, z):
        NewTarget = [label, x, y, z]
        InMemory = False
        for target in self.TargetList:
            # Determining how far something could be next to the said target,
            DistanceConfidence = math.sqrt(target[4]) * 1.5
            WithinX = abs(NewTarget[1] - target[1]) > DistanceConfidence
            WithinY = abs(NewTarget[2] - target[2]) > DistanceConfidence
            WithinZ = abs(NewTarget[3] - target[3]) > DistanceConfidence
            if (target[0] != NewTarget[0]) and WithinX and WithinY and WithinZ:
                InMemory = True
        return InMemory

    # handles the known targets in the surrounding area
    def SaveTargetToMemory(self, label, x, y, z, area):
        TargetInfo = [label, x, y, z, area]
        self.TargetList.append(TargetInfo)

    # handles checking parsed mission.txt with the movement_commands lists
    def CommandControl(self, commandline):
        self.Running = True

        # simple timer for controlling how long the command goes for. will be removed later on
        InitialTime = time.perf_counter()

        # booleans for telling command handler if it's a basic command or an advanced command
        BasicCMD = False
        AdvancedCMD = False
        ToTargetCMD = False

        # for testing - prints out on rpi terminal when ran
        print("Searching commands...")
        CommandCompilerIndex = 0
        CommandIndex = None
        SupplementaryCommand = ""
        # searching commands from mission.txt line
        for ParsedCommand in commandline.strip().split(','):
            if CommandCompilerIndex == 0:
                CommandType, CommandIndex = self.IsStringACommand(ParsedCommand)
                BasicCMD = (CommandType == "BASIC")
                AdvancedCMD = (CommandType == "ADVANCED")
                ToTargetCMD = (CommandType == "TARGET")
                CommandCompilerIndex += 1
            if CommandCompilerIndex == 1:
                SupplementaryCommand = ParsedCommand

        print("Running command: ", self.BASIC_MOVEMENT_COMMANDS[CommandIndex])
        time.sleep(1)
        try:
            while self.Running:
                # pixhawk updating
                if BasicCMD:
                    self.BasicCommand(CommandIndex)
                    ElapsedTime = time.perf_counter() - InitialTime
                    if ElapsedTime >= 60:
                        # for testing - prints out on rpi terminal when ran
                        print("End of command.")
                        self.BrakeAllThrusters()
                        self.Running = False
                if AdvancedCMD:
                    self.AdvancedCommand(CommandIndex, SupplementaryCommand)
        except:
            self.BrakeAllThrusters()
            self.Terminate()

    def UpdateThrusters(self):
        self.ThrusterLB.SetSpeed(self.PowerLB)
        self.ThrusterLF.SetSpeed(self.PowerLF)
        self.ThrusterRB.SetSpeed(self.PowerRB)
        self.ThrusterRF.SetSpeed(self.PowerRF)
        self.ThrusterBL.SetSpeed(self.PowerBL)
        self.ThrusterBR.SetSpeed(self.PowerBR)
        self.ThrusterFR.SetSpeed(self.PowerFR)
        self.ThrusterFL.SetSpeed(self.PowerFL)

    def BrakeAllThrusters(self):
        # horizontal
        self.PowerLB = 0
        self.PowerLF = 0
        self.PowerRB = 0
        self.PowerRF = 0
        # vert
        self.PowerBL = 0
        self.PowerBR = 0
        self.PowerFR = 0
        self.PowerFL = 0

        self.UpdateThrusters()

    def BasicCommand(self, commandnum, speed=17.5):
        if self.UsingPixHawk:
            self.PixHawk.UpdateGyro()
            self.PixHawk.CalculateError()
            self.PixHawk.PID()
        DownConst = -5.0
        # 0 = FORWARD
        if commandnum == 0:
            # horizontal
            self.PowerLB = speed
            self.PowerLF = speed
            self.PowerRB = speed
            self.PowerRF = speed
            # vert
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 1 = REVERSE
        if commandnum == 1:
            # horizontal
            self.PowerLB = -speed
            self.PowerLF = -speed
            self.PowerRB = -speed
            self.PowerRF = -speed
            # vert
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 2 = CLOCKWISE
        if commandnum == 2:
            self.PowerLB = speed
            self.PowerLF = -speed
            self.PowerRB = -speed
            self.PowerRF = speed

            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 3 = COUNTERCLOCKWISE
        if commandnum == 3:
            self.PowerLB = -speed
            self.PowerLF = speed
            self.PowerRB = speed
            self.PowerRF = -speed
            self.PowerBL = DownConst
            self.PowerBR = DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst

        # 4 = DIVE
        if commandnum == 4:
            self.PowerLB = 0.0
            self.PowerLF = 0.0
            self.PowerRB = 0.0
            self.PowerRF = 0.0

            self.PowerBL = speed
            self.PowerBR = speed
            self.PowerFR = -speed
            self.PowerFL = -speed

        # 5 = SURFACE
        if commandnum == 5:
            self.PowerLB = 0.0
            self.PowerLF = 0.0
            self.PowerRB = 0.0
            self.PowerRF = 0.0

            self.PowerBL = -speed
            self.PowerBR = -speed
            self.PowerFR = speed
            self.PowerFL = speed

        # 6 = IDLE
        if commandnum == 6:
            self.PowerLB = 0.0
            self.PowerLF = 0.0
            self.PowerRB = 0.0
            self.PowerRF = 0.0

            self.PowerBL = -DownConst
            self.PowerBR = -DownConst
            self.PowerFR = DownConst
            self.PowerFL = DownConst
        if self.UsingPixHawk:
            self.ThrusterLB.SetSpeedPID(self.PowerLB)
            self.ThrusterLF.SetSpeedPID(self.PowerLF)
            self.ThrusterRB.SetSpeedPID(self.PowerRB)
            self.ThrusterRF.SetSpeedPID(self.PowerRF)

            self.ThrusterBL.SetSpeedPID(self.PowerBL,
                                        rollpid=self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
            self.ThrusterBR.SetSpeedPID(self.PowerBR,
                                        rollpid=-self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
            self.ThrusterFL.SetSpeedPID(self.PowerFL,
                                        rollpid=-self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
            self.ThrusterFR.SetSpeedPID(self.PowerFR,
                                        rollpid=self.PixHawk.getRollPID(),
                                        pitchpid=-self.PixHawk.getPitchPID())
        else:
            self.UpdateThrusters()

    # using advanced movement and target
    def AdvancedCommand(self, commandnum, supplementarycmd):
        if self.UsingPixHawk:
            pass
        if commandnum == 0:
            pass
        if self.UsingPixHawk:
            self.PixHawk.UpdateGyro()
            self.PixHawk.CalculateError()
            self.PixHawk.PID()
            if commandnum == 5:
                # horizontal
                self.PowerLB = 0
                self.PowerLF = 0
                self.PowerRB = 0
                self.PowerRF = 0
                # vert
                self.PowerBL = 0
                self.PowerBR = 0
                self.PowerFR = 0
                self.PowerFL = 0


        else:
            print("Command requires the PixHawk")
            self.Running = False

    # ending vehicle connection and AI processing after mission completion/fatal error
    def Terminate(self):
        self.ThrusterLB.SetSpeed(0)
        self.ThrusterLF.SetSpeed(0)
        self.ThrusterRB.SetSpeed(0)
        self.ThrusterRF.SetSpeed(0)
        self.ThrusterBL.SetSpeed(0)
        self.ThrusterBR.SetSpeed(0)
        self.ThrusterFR.SetSpeed(0)
        self.ThrusterFL.SetSpeed(0)
        time.sleep(1)
        if self.UsingPixHawk:
            print("Killing Pixhawk...")
            time.sleep(1)
            self.PixHawk.Terminate()
        if self.UsingVision:
            print("Killing Vision...")
            time.sleep(1)
            self.VisionAI.terminate()
        print("Killing board...")
        time.sleep(1)
        board.exit()


def MapToPWM(x):
    in_min = -100.0
    in_max = 100.0
    out_min = 46.5
    out_max = 139.5
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def MapToSpeed(x):
    out_min = -100.0
    out_max = 100.0
    in_min = 46.5
    in_max = 139.5
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# dedicated class to driving a specific thruster
# has own PID, thruster, speed
class ThrusterDriver:
    def __init__(self, pin):
        connectstring = "d:" + str(pin) + ":s"
        self.thruster = board.get_pin(connectstring)
        print("Initializing Thruster: ", connectstring)
        self.speed = 93
        self.thruster.write(self.speed)

    # sets speed of thruster
    def SetSpeed(self, speed):  # speed is a value between -100 and 100
        self.speed = MapToPWM(speed)
        self.thruster.write(self.speed)

    #  sets speed of thruster and incorporates the addition of pwm variables
    def SetSpeedPID(self, speed, rollpid=0.0, pitchpid=0.0, yawpid=0.0):
        self.speed = float(float(speed) + float(rollpid) + float(pitchpid))
        self.speed = MapToPWM(self.speed)
        self.thruster.write(self.speed)

    # returns speed
    def GetSpeed(self):
        return self.speed
