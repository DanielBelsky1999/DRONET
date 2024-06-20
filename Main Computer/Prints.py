from colorama import init as colorama_init
from colorama import Fore
from colorama import Style
from threading import Lock

PrintLocker = Lock()
colorama_init()

def WARN(txt):
    with PrintLocker:
        print(f"\n{Fore.YELLOW} - WARNING:  " + str(txt) + f"{Style.RESET_ALL}")
    
def NOTIFY(txt):
    with PrintLocker:
        print(f"\n{Fore.GREEN} " + str(txt) + f"{Style.RESET_ALL}")
        
def SYSTEM_NOTIFY(txt):
    with PrintLocker:
        print(f"\n{Fore.CYAN} " + str(txt) + f"{Style.RESET_ALL}")

def ERROR(txt):
    with PrintLocker:
        print(f"\n{Fore.RED} x ERROR:  " + str(txt) + f"{Style.RESET_ALL}")
    
def DEBUG(txt):
    with PrintLocker:
        print(f"\n{Fore.LIGHTBLUE_EX} # DEBUG:  " + str(txt) + f"{Style.RESET_ALL}")
    
def PRINT_ROW(txt):
    with PrintLocker:
        print(f"{Fore.GREEN} " + str(txt) + f"{Style.RESET_ALL}")
    
def WHITE_PRINT(txt):
    with PrintLocker:
        print(f"{Fore.WHITE} " + str(txt) + f"{Style.RESET_ALL}") 