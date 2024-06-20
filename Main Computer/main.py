try:
    from GroundSystem import GroundSystem
except ModuleNotFoundError as e:
    import ctypes
    err_str = str(e)
    ctypes.windll.user32.MessageBoxW(None, err_str, u"MAIN COMPUTER ERROR", 0)
    exit()
    
import threading
from os import system

#####  MAIN  #####

system("cls") # clear screen

print("\n Press ctrl + C  to finish execution\n\n")

groundSystem = GroundSystem()

thread_main = threading.Thread(target=groundSystem.ReceiveAndProcessMessages)
thread_main.start()

thread_connection_checker = threading.Thread(target=groundSystem.ConnectionChecker)
thread_connection_checker.start()

try:
    input()
    while True:
        input("\n(Press ctrl + C  to finish execution)\n\n")
except KeyboardInterrupt:
    groundSystem.ShutDown()

thread_main.join()
thread_connection_checker.join()

input("\nPress ENTER to close the window\n")

##################
