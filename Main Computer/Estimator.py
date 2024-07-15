import numpy

class Estimator:
    
    def __init__(self):
        self.dT = 1/30 # Time interval

        self.currentTime = 0
        self.estimatedState = numpy.empty((9,1))
        
        print(self.estimatedState, end="\n\n")
        
        self.P_i = numpy.array([[1, self.dT, 0.5*self.dT**2],
                                [0, 1, self.dT],
                                [0, 0, 1]])
        
        print(self.P_i, end="\n\n")
        
        self.Phi_k = numpy.empty((9,9))
        self.Phi_k[0:3,0:3] = self.P_i
        self.Phi_k[3:6,3:6] = self.P_i
        self.Phi_k[6:9,6:9] = self.P_i
        
        print(self.Phi_k, end="\n\n")
        
        self.G = numpy.array([[0,0,0],
                            [0,0,0],
                            [1,0,0],
                            [0,0,0],
                            [0,0,0],
                            [0,1,0],
                            [0,0,0],
                            [0,0,0],
                            [0,0,1]])
                    
        print(self.G, end="\n\n")
        
        Q = (0.5)**2;     
        Q_G_transpose_dT = Q*numpy.transpose(self.G)*self.dT
        print(Q_G_transpose_dT, end="\n\n")
        self.Q_k = numpy.matmul(self.G, Q_G_transpose_dT)     
    
    
        print(self.Q_k, end="\n\n")

        self.z = numpy.empty((3,1))
        print(self.z, end="\n\n")
        
        self.H = numpy.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                             [0, 0, 0, 1, 0, 0, 0, 0, 0],
                             [0, 0, 0, 0, 0, 0, 1, 0, 0]])
        
        print(self.H, end="\n\n")
        
        self.measurement_sigma = 0.5;      
        self.R = numpy.diag(numpy.full(3,self.measurement_sigma**2))  
        print(self.R, end="\n\n")       
        self.z_tilde =  numpy.empty((3,1))
        print(self.z_tilde, end="\n\n")   
        
        
        
    def NewMeasurment(self, XYZt_solution):
        solution_time = XYZt_solution[3]
        
        
        while (self.currentTime < solution_time):
            #Predict
            if (self.currentTime == solution_time-self.dT):
                #Update:
            
            
            self.currentTime += self.dT;
            
            
        return #estimatedPositionMatrix[::]
        
        



if __name__ == "__main__":
    e = Estimator()
    
    xyzt = [4,5,6,7]
    returned_value = e.NewMeasurment(xyzt)
    