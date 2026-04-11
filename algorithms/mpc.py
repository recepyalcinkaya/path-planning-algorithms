import numpy as np
import math

class MPCConfig:
    def __init__(self):
        self.NX = 4  # State: [x, y, v, yaw]
        self.NU = 2  # Input: [accel, steer]
        self.T = 5   # Prediction horizon
        self.DT = 0.1 # Time step
        
        # Maliyet Ağırlıkları (Ar-Ge ekipleri bu değerleri 'tune' eder)
        self.Q = np.diag([1.0, 1.0, 0.5, 0.5]) # State tracking cost
        self.R = np.diag([0.01, 0.1])          # Control effort cost
        
        self.max_steer = 0.6  # [rad]
        self.max_accel = 1.0  # [m/s^2]

class MPCPlanner:
    def __init__(self, config=MPCConfig()):
        self.config = config

    def get_linear_model_matrix(self, v, phi, delta):
        # Aracın kinematik bisiklet modelinin lineerleştirilmiş hali
        # State space model: x(k+1) = A*x(k) + B*u(k)
        A = np.zeros((self.config.NX, self.config.NX))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.config.DT * math.cos(phi)
        A[0, 3] = - self.config.DT * v * math.sin(phi)
        A[1, 2] = self.config.DT * math.sin(phi)
        A[1, 3] = self.config.DT * v * math.cos(phi)
        A[3, 2] = self.config.DT * math.tan(delta) / 2.5 # 2.5: Wheelbase

        B = np.zeros((self.config.NX, self.config.NU))
        B[2, 0] = self.config.DT
        B[3, 1] = self.config.DT * v / (2.5 * math.cos(delta)**2)

        return A, B

    def predict_motion(self, x0, u):
        # Verilen kontrol girdilerine göre geleceği tahmin et
        x = np.zeros((self.config.NX, self.config.T + 1))
        x[:, 0] = x0
        for i in range(self.config.T):
            A, B = self.get_linear_model_matrix(x[2, i], x[3, i], u[1, i])
            x[:, i + 1] = A @ x[:, i] + B @ u[:, i]
        return x
