from GroundSystem import GroundSystem
import threading
from os import system


#####  MAIN  #####

system("cls") # clear screen

groundSystem = GroundSystem()

thread_main = threading.Thread(target=groundSystem.ReceiveAndProcessMessages)
thread_main.start()

thread_connection_checker = threading.Thread(target=groundSystem.ConnectionChecker)
thread_connection_checker.start()

try:
    while True:
        input("\n(ctrl + C  to finish execution)\n\n")
except KeyboardInterrupt:
    groundSystem.ShutDown()

thread_main.join()
thread_connection_checker.join()

input("Press ENTER to close the window")
##################
