import numpy as np
import math
from Prints import ERROR

class LeastSquaresSolver:

    def Solve(station_positions, LOS_vectors):
        
        if (len(station_positions) != len(LOS_vectors)):
            ERROR("Station Position matrix has different number of rows\n" + 
                  "  than LOS vectos matrix!!!" )
            return [0,0,0]
        
        a11 = a12 = a13 = a21 = a22 = a23 = a31 = a32 = a33 = d1 = d2 = d3 = 0
        for i in range(len(station_positions)):
            
            # Extract Position Vector components:
            p_x = station_positions[i,0]
            p_y = station_positions[i,1]
            p_z = station_positions[i,2]
            # Extract LOS Vector components: 
            v_x = LOS_vectors[i,0]
            v_y = LOS_vectors[i,1]
            v_z = LOS_vectors[i,2]
    
            # Matrix components calculation 
            a11=a11+2*v_z**2+2*v_y**2
            a12=a12-2*v_y*v_x
            a13=a13-2*v_z*v_x
            d1=d1+2*p_x*(v_z)**2-2*v_z*v_x*p_z+2*p_x*(v_y)**2-2*v_y*v_x*p_y
            a21=a21-2*v_y*v_x
            a22=a22+2*v_z**2+2*v_x**2
            a23=a23-2*v_z*v_y
            d2=d2+2*p_y*(v_z)**2-2*v_z*v_y*p_z-2*v_y*v_x*p_x+2*v_x**2*p_y
            a31=a31-2*v_z*v_x
            a32=a32-2*v_z*v_y
            a33=a33+2*v_y**2+2*v_x**2
            d3=d3-2*p_y*v_z*v_y+2*v_y**2*p_z-2*p_x*v_z*v_x+2*v_x**2*p_z
        
        
        # Construct the matrix and vector, and then calculate
        #    Ax = d   ---->   x = A^(-1)d   
        A = np.array([[a11, a12, a13], [a21, a22, a23], [a31, a32, a33]]) 
        d = np.array([d1, d2, d3])
        point = np.matmul(np.linalg.inv(A),d)
        
        X = point[0]
        Y = point[1]
        Z = point[2]
        return [X,Y,Z]


# EXAMPLE:
if __name__ == "__main__":

    # TEST 1
    
    # station_positions is a 4x3 matrix of the form:
    #  | 5  0  0 |    <-- station 1 position
    #  | 10 0  0 |    <-- station 2 position
    #  | 0  3  0 |    <-- station 3 position
    #  | 0 14  0 |    <-- station 4 position
    station_positions = np.array([[5,0,0], [10,0,0], [0,3,0], [0,14,0]])
    
    # LOS_vectors is a 4x3 matrix of the form:
    #  | 15 18 17 |    <-- station 1 LOS
    #  | 10 -2 17 |    <-- station 2 LOS
    #  | 20 15 17 |    <-- station 3 LOS
    #  | 20  4 17 |    <-- station 4 LOS
    LOS_vectors = np.array([[15,18,17], [10,18,17], [20,15,17], [20,4,17]])
    
    
    solution = LeastSquaresSolver.Solve(station_positions,LOS_vectors)
    print("\nTEST 1 - all LOS are intersecting at the target")
    print("Calculated Solution: " + str(solution))
    print("Real Solution is: " + str([20,18,17]))
    
    
    # TEST 2
    
    station_positions = np.array([[5,0,0], [11,0,1], [0,7,1], [0,21,2], [0,27,0]])
    LOS_vectors = np.array([[33,25,9], [26,27,8], [38,17,9], [36,6,7], [38,-1,8]])
    
    solution = LeastSquaresSolver.Solve(station_positions,LOS_vectors)
    print("\nTEST 2 - the LOS are NOT intersecting at the target")
    print("Calculated Solution: " + str(solution))
    print("Real Solution is: " + str([38.0178,25.8375,9.1568]))
    
    
    