import numpy as np
from Prints import *

class LosMatrix:
    
    def __init__(self):
        self.Reset() # Initializes empty matrix and array
        
    def AddLos(self, jetson_id, LOS):
        if (jetson_id not in self.corresponding_stations): # new LOS from jetson
            # Add jetson id to array:
            self.corresponding_stations = np.append(self.corresponding_stations,jetson_id)
            # Add LOS to matrix:
            self.matrix = np.vstack([self.matrix, LOS])
        
        else: # LOS from existing jetsons
            # Find the index of the corresponding row
            index_in_array = np.where(self.corresponding_stations==jetson_id)[0][0]
            # Replace the row with the new LOS vector
            self.matrix[index_in_array, :] = LOS
            
        
    def GetNumberOfRows(self):
        if (len(self.matrix) != len(self.corresponding_stations)):
            ERROR("WTF, the matrix has different number of rows than the array")
        return len(self.matrix)
    
    def GetMatrix(self):
        return self.matrix
    
    def GetJetsonIDsArray(self):
        return self.corresponding_stations
    
    def Reset(self):
        # init an empty 3 colomn matrix:
        self.matrix = np.empty([0, 3])   
        # init an empty array:
        self.corresponding_stations = np.array([])