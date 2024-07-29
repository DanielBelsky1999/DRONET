import numpy


class Estimator:

    def __init__(self):
        self.dT = 1 / 30  # Time interval
        self.currentTime = 0.
        self.estimatedState = numpy.zeros((9, 1))


        self.P_i = numpy.array([[1, self.dT, 0.5 * self.dT ** 2],
                                [0., 1, self.dT],
                                [0., 0., 1]])

        a = self.P_i.shape[0]
        self.Phi_k = numpy.zeros((9, 9))
        self.Phi_k[0:3, 0:3] = self.P_i
        self.Phi_k[3:6, 3:6] = self.P_i
        self.Phi_k[6:9, 6:9] = self.P_i


        self.G = numpy.array([[0, 0, 0],
                              [0, 0, 0],
                              [1, 0, 0],
                              [0, 0, 0],
                              [0, 0, 0],
                              [0, 1, 0],
                              [0, 0, 0],
                              [0, 0, 0],
                              [0, 0, 1]])


        Q = (0.5) ** 2
        self.Q_k = self.G @ self.G.T *self.dT*Q

        self.z = numpy.zeros((3, 1))
        
        self.H = numpy.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0, 0, 0, 0],
                              [0, 0, 0, 0, 0, 0, 1, 0, 0]])


        self.measurement_sigma = 0.5
        self.R = numpy.diag(numpy.full(3, self.measurement_sigma ** 2))
        self.z_tilde = numpy.zeros((3, 1))
        self.X_est = numpy.zeros((9, 1))
        self.X_est[0] = 50
        self.X_est[3] = 50
        self.P_est_i = numpy.array([[100, 0., 0.],
                                [0., 50, 0.],
                                [0., 0., 25]])
        self.P_est = numpy.zeros((9, 9))
        self.P_est[0:3, 0:3] = self.P_est_i
        self.P_est[3:6, 3:6] = self.P_est_i
        self.P_est[6:9, 6:9] = self.P_est_i

    def NewMeasurment(self, XYZ_solution, solution_time):
        measurement = numpy.array(XYZ_solution)
        row_vector = measurement.reshape(1, -1)
        self.z = row_vector.T
        Solution_matrix = numpy.empty([9, 0])
        time_vector = []
        epsilon = 10**-5
        while (abs(self.currentTime - solution_time) > epsilon):
            # Kalman filter run
            # Predict
            dim = len(self.X_est)
            # step 1: state prediction at time k+1|k
            self.X_est = numpy.dot(self.Phi_k, self.X_est)

            # step 2: state prediction error-covariance matrix at k+1|k
            P_pred = numpy.dot(numpy.dot(self.Phi_k, self.P_est), self.Phi_k.T) + self.Q_k

            if(abs(solution_time - self.currentTime - self.dT) < epsilon):
                # Update:
                # step 3: kalman gain calculation at time k+1
                S = numpy.dot(numpy.dot(self.H, P_pred), self.H.T) + self.R
                K = numpy.dot(numpy.dot(P_pred, self.H.T), numpy.linalg.inv(S))

                # step 4: measurement prediction at time k+1|k
                z_hat = numpy.dot(self.H, self.X_est)

                # innovations process z~ (k+1|k)
                self.z_tilde = self.z - z_hat

                # step 5: state estimation correction at time k+1|k+1
                self.X_est = self.X_est + numpy.dot(K, self.z_tilde)

                # Joseph formula for better accuracy
                self.P_est = numpy.dot(numpy.eye(dim) - numpy.dot(K, self.H), numpy.dot(P_pred, (numpy.eye(dim) - numpy.dot(K, self.H)).T)) + numpy.dot(
                    numpy.dot(K, self.R), K.T)
            # if (self.currentTime == solution_time-self.dT):
            self.currentTime += self.dT
            time_vector.append(self.currentTime)
            Solution_matrix = numpy.hstack([Solution_matrix, self.X_est])
            # add X_est to solution matrix

        return  Solution_matrix, time_vector


if __name__ == "__main__":
    e = Estimator()
    xyzt_0 = [10.49, 48, 0.05]
    xyzt_1 = [1, 7, 10]
    xyzt_2 = [1, 12, 10]
    print("in main")
    returned_value_0, time_0 = e.NewMeasurment(xyzt_0,(1/30))
    print(time_0)
    print("in main")
    returned_value_1, time_1 = e.NewMeasurment(xyzt_1, 3*(1/30))
    print(time_1)
    print("in main")
    returned_value_2, time_2 = e.NewMeasurment(xyzt_2, 10*(1/30))
    print(returned_value_2)
