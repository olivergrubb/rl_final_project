import numpy as np

class KalmanFilter:
    def __init__(self, dt, initial_state, initial_covariance):
        self.dt = dt
        self.state = initial_state
        self.covariance = initial_covariance

        # Define the state transition matrix
        self.A = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        
        # Define the control matrix
        self.B = np.array([[0.5 * dt**2, 0],
                           [0, 0.5 * dt**2],
                           [dt, 0],
                           [0, dt]])

        # Define the measurement matrix
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])

        # Define the process noise covariance matrix
        self.Q = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

        # Define the measurement noise covariance matrix
        self.R = np.array([[1, 0],
                           [0, 1]])

    def predict(self, control_input=np.array([0, 0])):
        # Predict the next state and covariance
        self.state = np.dot(self.A, self.state) + np.dot(self.B, control_input)
        self.covariance = np.dot(np.dot(self.A, self.covariance), self.A.T) + self.Q

    def update(self, measurement):
        # Update the state and covariance based on the measurement
        innovation = measurement - np.dot(self.H, self.state)
        innovation_covariance = np.dot(np.dot(self.H, self.covariance), self.H.T) + self.R
        kalman_gain = np.dot(np.dot(self.covariance, self.H.T), np.linalg.inv(innovation_covariance))

        self.state = self.state + np.dot(kalman_gain, innovation)
        self.covariance = np.dot((np.eye(4) - np.dot(kalman_gain, self.H)), self.covariance)
