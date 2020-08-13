import movement_commander_v3
import time

testthruster = ThrusterDriver(3)

testthruster.SetSpeed(20)
time.sleep(5)
testthruster.SetSpeed(0)
