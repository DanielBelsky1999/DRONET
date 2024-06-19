from Stations import Stations
from Origin import Origin
from Logger import Logger
from Calibrator import Calibrator
from ConnectionChecker import ConnectionChecker
from LeastSquaresSolver import LeastSquaresSolver
import socket
from Prints import *
from GuiSender import GuiSender

class GroundSystem:
    
    # System Configuration
    TOTAL_NUM_OF_STATIONS = 6                     # Total Number of stations in the system
    NUM_OF_STATIONS_FOR_CALCULATION = 4     # A calculation is performed when this number of LOS arrive
    CONFIGURATION_MIN_NUM_OF_STATIONS = 4   # Below this number of stations - the system will not count as "configured"
    
    
    # Constants
    UDP_IP = "0.0.0.0"
    UDP_PORT = 25001
    
    # Calibrator Constants
    CAMERA_NUM = 1
    LENS_NUM = 1
      
      
    # Constructor
    def __init__(self):
        GroundSystem.PrintWelcome()
    
        self.DONT_LAUNCH_THREAD = False
        self.stopConnectionChecker = False
        
        self.stations = Stations(GroundSystem.TOTAL_NUM_OF_STATIONS)
        self.Origin = Origin()
        
        # Logger - if it errors - close everything
        try:
            self.logger = Logger(GroundSystem.TOTAL_NUM_OF_STATIONS, LogMATLAB = True, LogCSV = True, TestLog = True)
        except PermissionError:
                ERROR("Close all opened log files !!!! \n    Then Restart main !!\n")
                self.DONT_LAUNCH_THREAD = True
                return
        #
        
        self.calibrator = Calibrator(GroundSystem.CAMERA_NUM, GroundSystem.LENS_NUM)   
        self.SetUpSocket()  
        self.connectionChecker = ConnectionChecker(GroundSystem.TOTAL_NUM_OF_STATIONS)
        self.XYZ_solution = [0,0,0]
        self.systemIsConfigured = False
        self.OriginIsSet = False
        
        GuiSender.SendRESET()
        
        
    def SetUpSocket(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.server_socket.bind((GroundSystem.UDP_IP, GroundSystem.UDP_PORT))
        except OSError:
            ERROR("     OSError. Make Sure port \'{port}\' is free".format(port=GroundSystem.UDP_PORT))
            self.DONT_LAUNCH_THREAD = True
            
    def ShutDown(self):
        NOTIFY("###### Closing Everyting.....  ")
        self.stopConnectionChecker = True
        if (not self.DONT_LAUNCH_THREAD):
            self.server_socket.shutdown(socket.SHUT_RDWR)
            self.server_socket.close()
        else:
            DEBUG("server_socket wasn't initialized")
            
        self.logger.CloseAll()
    
    def ReceiveAndProcessMessages(self):
        if (self.DONT_LAUNCH_THREAD == False):
            NOTIFY("Waiting for packets ... ")
            while True:
                try:
                    raw_data = self.RecvMessage()
                except OSError: # socket was closed:
                    NOTIFY("Socket closed")
                    break
                data_packets = raw_data.split(";")
                for packet in data_packets[0:-1]: # exclude last cell which is an empty string
                    self.ParseMessage(packet)
        
    def RecvMessage(self):
        bytesAddressPair = self.server_socket.recvfrom(1024)
        message = bytesAddressPair[0]
        # address = bytesAddressPair[1]
        raw_data = message.decode()
        return raw_data
        
    def ParseMessage(self, packet):
        packet_fields = packet.split(",")
        packet_HEADER = packet_fields[0]
        packet_BODY = packet_fields[1:]
        # DEBUG(packet_fields)

        if packet_HEADER == "GUI":
            self.ParseGuiMessage(packet_BODY)
        elif packet_HEADER == "STATION":
            self.ParseStationMessage(packet_BODY)
        elif packet_HEADER == "TEST":
            print(packet)
            self.ParseTestMessage(packet_BODY)
            
    def ParseGuiMessage(self, split_message):
        MESSAGE_HEADER = split_message[0]
        if MESSAGE_HEADER == "POS":
            split_message = split_message[1:] # Cut off the message header
            try:
                jetson_id = int(split_message[0])
                lat = float(split_message[1])
                lon = float(split_message[2])
                alt = float(split_message[3])
                psi = float(split_message[4])
                theta = float(split_message[5])
                phi = float(split_message[6])     

                if (jetson_id == 0):
                    self.SetOrigin(lat, lon, alt, psi, theta, phi)
                else:     
                    self.SetStation(jetson_id, lat, lon, alt, psi, theta, phi)
            
            except ValueError:
                WARN("Something went wrong with conversion (str)->(float). \n"
                     +"         Make sure you have filled all the fields.")
        
        elif MESSAGE_HEADER == "UPDATE_ME":
            self.GuiFullUpdate()
            
    def ParseStationMessage(self, split_message):
        try:
            jetson_id = int(split_message[0])
            pixel_1 = float(split_message[1])
            pixel_2 = float(split_message[2])
            
            notify_gui_new_station = self.connectionChecker.NewMessage(jetson_id)
            if (notify_gui_new_station):
                GuiSender.SendConnectionStatus(jetson_id, "CONNECTED")
            
            if self.systemIsConfigured: # Parse these messages only if the system is configured
                self.ProcessStationMessage(jetson_id, pixel_1, pixel_2)
        
        
        except ValueError:
            ERROR("Something Wrong with Conversion (str)->(float)")
        
    def SetOrigin(self, lat, long, alt, psi, theta, phi):
        if (self.OriginIsSet == False):
            self.Origin.SetPositionLLA(lat, long, alt)
            self.Origin.SetOrientationRelativeToNED(psi, theta, phi)
            self.OriginIsSet = True
            self.logger.RecordOrigin(self.Origin)
            GuiSender.SendOriginSet()
            NOTIFY("Origin Set Successfully!")
        else:
            WARN("Origin was already set. The LLH position of the origin is {lon}, {lat}, {alt}".format(lon = self.Origin.longtitude,
                                                                                                        lat = self.Origin.latitude,
                                                                                                        alt = self.Origin.altitude))
                                                                                                        
    def SetStation(self, jetson_id, lat, long, alt, yaw_left, pitch_up, roll_right): 
        if (self.OriginIsSet == False):
            WARN("You MUST set the origin first. \n             To do this - enter station No. zero (0)")
        else:
            ## HANLDE NEW STATION ##
            # 1. Set Station Position in Origin Coordinate system
            set_pos_successfully = self.stations.SetJetsonPositionInOriginFromLLA(jetson_id,
                                                                                self.Origin,
                                                                                lat,
                                                                                long,
                                                                                alt)
                                                            
            # 2. Set Station Orientation relative to ORIGIN Coordinate system, given ABSOLUTE angles
            # set_orientation_successfully = self.stations.SetJetsonOrientationRelativeToOriginSystem_ABSOLUTE_Angles(jetson_id, 
                                                                                                                # self.Origin, 
                                                                                                                # azimuth_rel_to_north, 
                                                                                                                # pitch_up, 
                                                                                                                # roll_right)
            # 2. Set Station Orientation relative to ORIGIN Coordinate system, given RELATIVE angles
            set_orientation_successfully = self.stations.SetJetsonOrientationRelativeToOriginSystem_RELATIVE_Angles(jetson_id, 
                                                                                                                yaw_left, 
                                                                                                                pitch_up, 
                                                                                                                roll_right)
            
            
            if (set_pos_successfully and set_orientation_successfully):
                
                # 3. Increase the number of confugured stations
                self.stations.SetStationConfigured(jetson_id)
                
                # 4. Notify in Console
                self.stations.PrintJetsonStation(jetson_id)
                #self.stations.PrintAllJetsonStations()

                # 5. Send GUI message
                POS = self.stations.GetJetsonPosXY_forGUI(jetson_id)
                LOS = self.stations.GetJetsonMainAxisXY_forGUI(jetson_id)
                GuiSender.SendJetsonPosition(jetson_id, POS, LOS)
                GuiSender.SendJetsonPositionStatus(jetson_id, "SET")
                
                # 6. Record to Log Files
                self.logger.RecordStation(jetson_id, self.stations.stations_array[jetson_id-1])
            
            # 7. Check if the system is configured:
            if (self.stations.GetNumberOfConfiguredStations() == GroundSystem.CONFIGURATION_MIN_NUM_OF_STATIONS):
                self.systemIsConfigured = True
                NOTIFY("Ground System Configured")
                
    def ProcessStationMessage(self, jetson_id, pixel_1, pixel_2):
        # 1. Clibrate:  
        #    pixels ---> azimuth, elevation
        azimuth_deg, elevation_deg = self.calibrator.Calibrate(jetson_id, pixel_1, pixel_2)
        
        # 2. Translate to LOS in origin frame:  
        #    azimuth, elevation ---> LOS in origin frame
        LOS = self.stations.GetLOSinOriginFrame(jetson_id, azimuth_deg, elevation_deg)
        # 2.1 Record the LOS
        self.logger.RecordLOS(jetson_id, LOS)
        
        # # 3. Add to LOS matrix
        self.stations.AddLOS(jetson_id, LOS)
        
        # # 4. Check Matrix - if it has "enough" rows - Calculate Solution! 
        if (self.stations.GetNumberOfLOS_rows() == GroundSystem.NUM_OF_STATIONS_FOR_CALCULATION):
            # 4.1 Get the matrices
            position_matrix, LOS_matrix = self.stations.GetMatricesForSolver()            
            # 4.2 Pass to Least Squares Solver:
            self.XYZ_solution = LeastSquaresSolver.Solve(position_matrix, LOS_matrix)
            # 4.3 Record Solution
            self.logger.RecordSolution(self.XYZ_solution)
            
            # 5. Estimator:
            # TODO
            
            # 6. Send to GUI
            x = self.XYZ_solution[0]
            y = self.XYZ_solution[1]
            z = self.XYZ_solution[2]
            GuiSender.SendXYZ(x,y,z)
            
            
    def GuiFullUpdate(self):      
        if (self.OriginIsSet):
            GuiSender.SendOriginSet()
            for i,jetson in enumerate(self.stations.stations_array):
                if (jetson.configured):
                    jetson_id = i + 1
                    POS = self.stations.GetJetsonPosXY_forGUI(jetson_id)
                    LOS = self.stations.GetJetsonMainAxisXY_forGUI(jetson_id)
                    GuiSender.SendJetsonPosition(jetson_id, POS, LOS)
                    GuiSender.SendJetsonPositionStatus(jetson_id, "SET")
                    
                                       
    def ParseTestMessage(self, split_message):
        px_x = int(split_message[0])
        px_y = int(split_message[1])
        GuiSender.SendXYZ(px_x,px_y,0)
        
        # Calibrate 
        azimuth_deg, elevation_deg = self.calibrator.Calibrate(0, px_x, px_y)

        # LOG PIXELS
        self.logger.RecordTestMessage(px_x, px_y, azimuth_deg, elevation_deg)    
    
    def ConnectionChecker(self):
        while True:
            if not self.stopConnectionChecker:
                jetsons_disconnected = self.connectionChecker.Check()
                for jetson_id in jetsons_disconnected:
                    GuiSender.SendConnectionStatus(jetson_id, "DISCONNECTED")
                    WARN("Jetson {i} Disconnected".format(i=jetson_id))
            else:
                break
        NOTIFY("Connection Checker Thread END\n")
        
        
    def PrintWelcome():
        space = " "*10
        print()
        print()
        print(space + "##########################")
        print(space + "##########################")
        print(space + "###                    ###")
        print(space + "###      DRONET®       ###")
        print(space + "###                    ###")
        print(space + "###    Main Computer   ###")
        print(space + "###       (beta)       ###")
        print(space + "###                    ###")
        print(space + "###                    ###")
        print(space + "##########################")
        print(space + "##########################")
        print()
        