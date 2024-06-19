from DCM import DCM

class Origin:
    
    def __init__(self):
        # initializes to 
        # Faculty of Aerospace Engineering, Technion (because why not..):
        self.longtitude = 35.02168729438545
        self.latitude = 32.77429928807602
        self.altitude = 237
        self.orientation = DCM()
    
    def SetOrientationRelativeToNED(self, azimuth, pitch_up, roll_right):
        self.orientation.SetDCMfromEuler(azimuth, pitch_up, roll_right + 180)
    
    def SetPositionLLA(self, lat, long, alt):
        self.latitude = lat
        self.longtitude = long
        self.altitude = alt

