try:
    from GroundSystem import GroundSystem
    from Prints import CYAN_PRINT
except ModuleNotFoundError as e:
    import ctypes
    err_str = str(e)
    ctypes.windll.user32.MessageBoxW(None, err_str, u"MAIN COMPUTER ERROR", 0)
    exit()
    
import threading
from os import system

#####  MAIN  #####

system("cls") # clear screen

CYAN_PRINT("\n Press ctrl + C  to finish execution\n\n")

groundSystem = GroundSystem()

thread_main = threading.Thread(target=groundSystem.ReceiveAndProcessMessages)
thread_main.start()

thread_connection_checker = threading.Thread(target=groundSystem.ConnectionChecker)
thread_connection_checker.start()

try:
    input()
    while True:
        CYAN_PRINT("\n (Press ctrl + C  to finish execution)\n\n")
        input()
except KeyboardInterrupt:
    groundSystem.ShutDown()

thread_main.join()
thread_connection_checker.join()

CYAN_PRINT("\n   Press ENTER to close the window\n")
input()
##################
