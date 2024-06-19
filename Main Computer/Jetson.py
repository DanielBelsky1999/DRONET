import numpy
import navpy
from DCM import DCM
from Origin import Origin

class Jetson:
    
    def __init__(self):
        self.position = numpy.array([0,0,0])
        self.orientation = DCM()  # <---- DCM FROM ORIGIN TO STATION !!!
        self.configured = False
        
    def SetPositionXYZ(self, x,y,z):
        self.position = numpy.array([x,y,z])
        
    def SetJetsonPositionInOriginFromOriginAndLLA(self, origin, lat, long, alt):
        N,E,D = navpy.lla2ned(lat, long, alt,
                              origin.latitude, origin.longtitude, origin.altitude, 
                              latlon_unit='deg', alt_unit='m', model='wgs84')
                              
        DCM_ned2origin = origin.orientation.GetMatrix()
        ned_vector = numpy.array([N,E,D])           
        origin_vector = numpy.matmul(DCM_ned2origin, ned_vector)
        self.SetPositionXYZ(origin_vector[0,0],origin_vector[0,1],origin_vector[0,2])
    
    def SetJetsonOrientationRelativeToOriginSystem_ABSOLUTE_Angles(self, origin, azimuth, pitch_up, roll_right):
        # This sets orientation to be DCM FROM ORIGIN TO STATION !!!  
        DCM_ned2station = DCM()
        DCM_ned2station.SetDCMfromEuler(azimuth, pitch_up, roll_right + 180)
        
        DCM_Origin2ned = DCM.Transpose(origin.orientation)
        # The order is really very super important. Don't ask me how I found out.
        self.orientation = DCM.Multiply2DCMs(DCM_ned2station, DCM_Origin2ned)
    
    def SetJetsonOrientationRelativeToOriginSystem_RELATIVE_Angels(self, Yaw_left, Pitch_up, Roll_right):
        # Watch out for the minus - since rotation "pitch-up" is against right rand rule 
        self.orientation.SetDCMfromEuler(Yaw_left, -Pitch_up, Roll_right)
    
    def GetOrientation(self):
        return self.orientation
    
    def GetMainAxisXY_forGUI(self):
        mat = (self.orientation.GetMatrix()).transpose()
        return [mat[0,0], mat[1,0]]
        
        
    def GetLOS_origin_frame(self, azimuth_deg, elevation_deg):
        dcm_station2LOS = DCM()
        dcm_station2LOS.SetDCMfromEuler(-azimuth_deg, -elevation_deg, 0)  
        matrix_station2LOS = dcm_station2LOS.GetMatrix()
        matrix_LOS2station = matrix_station2LOS.transpose()
        
        matrix_origin2station = self.orientation.GetMatrix()
        matrix_station2origin = matrix_origin2station.transpose()
        
        # The order of multiplication is important !
        matrix_LOS2origin = numpy.matmul(matrix_station2origin, matrix_LOS2station)
        return matrix_LOS2origin.A[:,0]  # first column, as if the matrix is multiplied by [1;0;0]

        
if __name__ == "__main__":
    print("--- TEST JETSON ---")
    
    print("--------------------")