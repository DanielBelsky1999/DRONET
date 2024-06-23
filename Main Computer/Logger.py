import os

from CONFIG import CONFIG_Logger
from Origin import Origin
from Jetson import Jetson
from Prints import *

class Logger:

    HFileSystemLogMATLAB = None
    HFileDataLogMATLAB = None
    HFileSystemLogCSV = None
    HFileTestLogMATLAB = None
    
    LogMATLAB = True
    LogCSV = True
    TestLog = False
    
    def __init__(self, num_of_stations, LogMATLAB = True, LogCSV = True, TestLog = False):
        Logger.LogMATLAB = LogMATLAB
        Logger.LogCSV = LogCSV
        Logger.TestLog = TestLog
        
        MATLAB_SystemLog_path = CONFIG_Logger.LogFolderDir + CONFIG_Logger.SystemLogFileMATLAB_name
        MATLAB_DataLog_path = CONFIG_Logger.LogFolderDir + CONFIG_Logger.DataLogFileMATLAB_name
        CSV_log_path = CONFIG_Logger.LogFolderDir + CONFIG_Logger.SystemLogFileCSV_name
        Test_log_path = CONFIG_Logger.LogFolderDir + CONFIG_Logger.TestLogFileMATLAB_name
        
        if (not os.path.exists(CONFIG_Logger.LogFolderDir)):
            os.mkdir(CONFIG_Logger.LogFolderDir)
            NOTIFY("Created Folder: " + CONFIG_Logger.LogFolderDir)
        
        if (LogMATLAB):
            Logger.HFileSystemLogMATLAB = open(MATLAB_SystemLog_path, "w")
            Logger.HFileDataLogMATLAB = open(MATLAB_DataLog_path, "w")
        if (LogCSV):
            Logger.HFileSystemLogCSV = open(CSV_log_path, "w")
        if (TestLog):
            Logger.HFileTestLogMATLAB = open(Test_log_path, "w")
        
        # Initialize a histogram to count how many times a jetson has sent it's LOS     
        self.jetson_LOS_histogram = [1 for i in range(num_of_stations)]
        # Initialize a solution counter
        self.solution_index = 1
        
        ### For Testing ###
        self.test_msg_indx = 1
        
        
                           
    def CloseAll(self):
        if (Logger.LogMATLAB):
            try:
                Logger.HFileSystemLogMATLAB.close()
                PRINT_ROW(CONFIG_Logger.SystemLogFileMATLAB_name + " Closed successfully")
                Logger.HFileDataLogMATLAB.close()
                PRINT_ROW(CONFIG_Logger.DataLogFileMATLAB_name + " Closed successfully")
            except AttributeError:
                pass
        if (Logger.LogCSV):
            try:
                Logger.HFileSystemLogCSV.close()
                PRINT_ROW(CONFIG_Logger.SystemLogFileCSV_name + " Closed successfully")       
            except AttributeError: 
                pass
        if (Logger.TestLog):
            try:
                Logger.HFileTestLogMATLAB.close()
                PRINT_ROW(CONFIG_Logger.TestLogFileMATLAB_name + " Closed successfully")       
            except AttributeError: 
                pass       
                
    def RecordOrigin(self, origin):
        if (Logger.LogMATLAB):
            new_line = "; \n"
            txt =  " % Origin" + new_line
            txt += " Origin.latitude = " + str(origin.latitude) + new_line
            txt += " Origin.longtitude = " + str(origin.longtitude) + new_line
            txt += " Origin.altitude = " + str(origin.altitude) + new_line
            mat = origin.orientation.GetMatrix()
            txt += " Origin.DCM_ned2Origin = [{a11}, {a12}, {a13}".format(a11 = mat[0,0], a12 = mat[0,1], a13 = mat[0,2]) + new_line
            txt += "                         {a21}, {a22}, {a23}".format(a21 = mat[1,0], a22 = mat[1,1], a23 = mat[1,2]) + new_line
            txt += "                         {a31}, {a32}, {a33}]".format(a31 = mat[2,0], a32 = mat[2,1], a33 = mat[2,2]) + new_line
            txt += "\n"
            Logger.HFileSystemLogMATLAB.write(txt)
            
        if (Logger.LogCSV):
            new_line = "\n"
            txt = "0" + new_line
            txt += ",".join([str(origin.latitude),str(origin.longtitude),str(origin.altitude)]) + new_line
            mat = origin.orientation.GetMatrix()
            txt += ",".join([str(mat[i,j]) for i in range(3) for j in range(3)]) + new_line
            Logger.HFileSystemLogCSV.write(txt)
            
    def RecordStation(self, jetson_id, jetson):
        if (Logger.LogMATLAB):
            new_line = "; \n"
            txt =  " % Station " + str(jetson_id) + new_line
            txt += " Stations.position({i},:) = ".format(i = jetson_id) + str(jetson.position) + new_line
            mat = jetson.orientation.GetMatrix()
            txt += " Stations.Origin2StationDCM({i},:,:) = [{a11}, {a12}, {a13}".format(i = jetson_id, a11 = mat[0,0], a12 = mat[0,1], a13 = mat[0,2]) + new_line
            txt += "                                        {a21}, {a22}, {a23}".format(               a21 = mat[1,0], a22 = mat[1,1], a23 = mat[1,2]) + new_line
            txt += "                                        {a31}, {a32}, {a33}]".format(              a31 = mat[2,0], a32 = mat[2,1], a33 = mat[2,2]) + new_line
            txt += "\n"
            Logger.HFileSystemLogMATLAB.write(txt)
            
        if (Logger.LogCSV):
            new_line = "\n"
            txt = str(jetson_id) + new_line
            txt += ",".join([str(jetson.position[i]) for i in range(3)]) + new_line
            mat = jetson.orientation.GetMatrix()
            txt += ",".join([str(mat[i,j]) for i in range(3) for j in range(3)]) + new_line
            Logger.HFileSystemLogCSV.write(txt)
    
    def RecordLOS(self, jetson_id, LOS):
        if (Logger.LogMATLAB):
            new_line = "; \n"
            jetson_LOS_indx = self.jetson_LOS_histogram[jetson_id-1]
            txt = "LOS({i},{indx},:) = ".format(i=jetson_id, 
                                                indx = jetson_LOS_indx)
            txt += "[{i},{j},{k}]".format(i=LOS[0], j=LOS[1], k=LOS[2]) + new_line
            self.jetson_LOS_histogram[jetson_id-1] = jetson_LOS_indx + 1  
            Logger.HFileDataLogMATLAB.write(txt)
    
    def RecordSolution(self, XYZ_array):
        if (Logger.LogMATLAB):
            new_line = "; \n"
            solution_indx = self.solution_index
            txt = "Solution({i},:) = ".format(i=self.solution_index)
            txt += "[{i},{j},{k}]".format(i=XYZ_array[0], j=XYZ_array[1], k=XYZ_array[2]) + new_line
            self.solution_index = solution_indx + 1  
            Logger.HFileDataLogMATLAB.write(txt)
        
        
    def RecordTestMessage(self, px1, px2, azimuth_deg, elevation_deg):
        if (Logger.LogMATLAB):
            new_line = "; \n"
            txt = "pixels({i},:) = [{px_x},{px_y}]".format(i = self.test_msg_indx, px_x=px1, px_y=px2) + new_line;
            txt += "Calibration_result({i},:) = [{azimuth},{pitch}]".format(i = self.test_msg_indx, azimuth=azimuth_deg, pitch=elevation_deg) + new_line;
            self.test_msg_indx += 1
            Logger.HFileTestLogMATLAB.write(txt)