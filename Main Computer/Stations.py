from Jetson import Jetson
from LosMatrix import LosMatrix
from Origin import Origin
from Prints import * 

import numpy

class Stations:
    
    def __init__(self, num_of_stations):
        self.stations_array = [Jetson() for i in range(num_of_stations)]
        self.LOS_matrix = LosMatrix()
        self.number_of_stations = num_of_stations
        
    def SetJetsonPositionXYZ(self, jetson_id, x,y,z):
        if (self.CheckID(jetson_id)):
            self.stations_array[jetson_id - 1].SetPositionXYZ(x,y,z)
            return True
        return False
    
    def SetJetsonPositionInOriginFromLLA(self, jetson_id, origin, lat, long, alt):
        if (self.CheckID(jetson_id)):  
            self.stations_array[jetson_id - 1].SetJetsonPositionInOriginFromOriginAndLLA(origin, lat, long, alt)
            return True
        return False
    
    def SetJetsonOrientationRelativeToOriginSystem_ABSOLUTE_Angles(self, jetson_id, origin, azimuth, pitch_up, roll_right):
        if (self.CheckID(jetson_id)): 
            self.stations_array[jetson_id - 1].SetJetsonOrientationRelativeToOriginSystem_ABSOLUTE_Angles(origin, azimuth, pitch_up, roll_right)
            return True
        return False
    
    def SetJetsonOrientationRelativeToOriginSystem_RELATIVE_Angles(self, jetson_id, Yaw_left, Pitch_up, Roll_right):
        if (self.CheckID(jetson_id)): 
            self.stations_array[jetson_id - 1].SetJetsonOrientationRelativeToOriginSystem_RELATIVE_Angels(Yaw_left, Pitch_up, Roll_right)
            return True
        return False
            
    def CheckID(self, jetson_id):
        # jetson_id should be in range (1,2, ... , number_of_stations)
        if (jetson_id < 1 or jetson_id > self.number_of_stations):
            WARN("Jetson-Station {i} doesn't exist. Position NOT SET".format(i=jetson_id))
            return False
        return True
    
    def GetJetsonPosXY_forGUI(self, jetson_id):
        jetson_i = self.stations_array[jetson_id - 1]
        return [jetson_i.position[0], jetson_i.position[1]]
        
    def GetJetsonMainAxisXY_forGUI(self, jetson_id):
        jetson_i = self.stations_array[jetson_id - 1]
        return jetson_i.GetMainAxisXY_forGUI()
        
    def GetLOSinOriginFrame(self, jetson_id, azimuth_deg, elevation_deg):
        LOS = self.stations_array[jetson_id - 1].GetLOS_origin_frame(azimuth_deg, elevation_deg)
        return LOS
    
    def SetStationConfigured(self, jetson_id):
        self.stations_array[jetson_id - 1].configured = True
        
    def GetNumberOfConfiguredStations(self):
        num = 0
        for jetson in self.stations_array:
            num += int(jetson.configured)
        return num
    
    def AddLOS(self, jetson_id, LOS):
        # Check if the jetson was configured:
        if (self.stations_array[jetson_id-1].configured == False):
            WARN("Got message from an unconfigured jetson station at id: {i}".format(i=jetson_id))
        else: # Jetson Configured:
            self.LOS_matrix.AddLos(jetson_id, LOS)
            
    
    def GetNumberOfLOS_rows(self):
        return self.LOS_matrix.GetNumberOfRows()
       
    def GetMatricesForSolver(self):
        # position matrix
        jetson_indexes = self.LOS_matrix.GetJetsonIDsArray()
        position_mat = numpy.empty([0, 3])
        for jetson_np_id in jetson_indexes:
            jetson_id = int(jetson_np_id)
            position_mat = numpy.vstack([position_mat, self.stations_array[jetson_id-1].position])
        
        # LOS matrix
        LOS_mat = self.LOS_matrix.GetMatrix()
        
        # Reset matrices
        self.LOS_matrix.Reset()
        
        return position_mat, LOS_mat
        
    
    def PrintJetsonStation(self, jetson_id): # id should be in range 1-6
        jetson_i = self.stations_array[jetson_id - 1]
        euler_angles = jetson_i.orientation.GetEulerAnglesDeg()
        print("\nStation{i}:".format(i=jetson_id))
        print("Position: {position_np_arr}".format(position_np_arr =
                                                       jetson_i.position))
        print("Euler Angles RELATIVE TO ORIGIN SYSTEM:\n"
        "   (psi,theta,phi) = ({psi:.4f}, {theta:.4f}, {phi:.4f})".format(psi=euler_angles[0],
                                                                        theta=euler_angles[1],
                                                                        phi=euler_angles[2]))
    def PrintAllJetsonStations(self):
        print("- JETSON STATIONS DATA:")
        for jetson_id in range(self.number_of_stations):
            self.PrintJetsonStation(jetson_id + 1)
            
            


if __name__ == "__main__":
    print("--- TEST STATIONS ---")
    
    print("---------------------")