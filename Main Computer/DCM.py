import numpy
import math

numpy.set_printoptions(precision=10)

class DCM:
    
    # Static:
    DCM_180_roll = numpy.matrix([[1,0,0],
                                 [0,-1,0],
                                 [0,0,-1]])
    def Multiply2DCMs(A,B):
        C = numpy.matmul(A.GetMatrix(), B.GetMatrix())
        tempDCM = DCM()
        tempDCM.rotation_matrix = C
        return tempDCM
        
    def Transpose(A):
        B = A.GetMatrix().transpose()
        tempDCM = DCM()
        tempDCM.rotation_matrix = B
        return tempDCM
    
    
    # Instance Related:    
    def __init__(self):
        self.rotation_matrix = numpy.matrix([[1,0,0],
                                             [0,1,0],
                                             [0,0,1]])                         
    
    def SetDCMfromEuler(self, psi_deg, theta_deg, phi_deg):
        # deg to rad
        d2r = math.pi/180
        psi = psi_deg*d2r
        theta = theta_deg*d2r
        phi = phi_deg*d2r
        
        
        # Construct DCM
        cs = math.cos(psi)
        ss = math.sin(psi)
        ct = math.cos(theta)
        st = math.sin(theta)
        cp = math.cos(phi)
        sp = math.sin(phi)
        
        self.rotation_matrix = numpy.matrix([[ct*cs         , ct*ss          , -st  ], 
                                            [sp*st*cs-cp*ss , sp*st*ss+cp*cs , sp*ct],
                                            [cp*st*cs+sp*ss , cp*st*ss-sp*cs , cp*ct]])
    
    def GetEulerAnglesDeg(self):
        r2d = 180/math.pi
    
        C12 = self.rotation_matrix.A[0,1]
        C11 = self.rotation_matrix.A[0,0]
        psi = math.atan2(C12,C11) * r2d
        
        C13 = self.rotation_matrix.A[0,2]
        theta = -math.asin(C13) * r2d
        
        C23 = self.rotation_matrix.A[1,2]
        C33 = self.rotation_matrix.A[2,2]
        phi = math.atan2(C23,C33) * r2d
        
        return psi,theta,phi
    
    def GetMatrix(self):
        return self.rotation_matrix




if __name__ == "__main__":
    print("--- TEST DCM ---")
    
    print("--------------------")