
##################################################
#
#    Here you config everything you need!
#       - Ground System
#       - Calibrator
#       - Connection Checker
#       - Logger
#
##################################################

# System Configuration
class CONFIG_GroundSystem:
    
    TOTAL_NUM_OF_STATIONS = 6                     # Total Number of stations in the system
    NUM_OF_STATIONS_FOR_CALCULATION = 4     # A calculation is performed when this number of LOS arrive
    CONFIGURATION_MIN_NUM_OF_STATIONS = 4   # Below this number of stations - the system will not count as "configured"
    
    # Communication Socket:
    UDP_IP = "0.0.0.0"
    UDP_PORT = 25001
  

# Calibrator Configuration 
class CONFIG_Calibrator:
    # Camera NUM:
    # 1 - .....
    # 2 - .....
    # 3 - .....
    CAMERA_NUM = 1
    
    # Lens NUM:
    # 1 - .....
    # 2 - .....
    # 3 - .....
    LENS_NUM = 1
    
    
# Connection Checker configuration
class CONFIG_ConnectionChecker:
    # This is the time-out for each jetson before it will be showd as "disconnect"
    WAIT_TIME = 5  # [seconds]
    
    
# Logger Configuration
class CONFIG_Logger:
    LogFolderDir = "Logs\\"
    SystemLogFileMATLAB_name = "SystemLog.m"
    DataLogFileMATLAB_name = "DataLog.m"
    TestLogFileMATLAB_name = "test.m"
    SystemLogFileCSV_name = "SystemLog.csv"