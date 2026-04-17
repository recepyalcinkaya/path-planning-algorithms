import math
import numpy as np

class LQRController:
    def __init__(self):
        # Ar-Ge Kalibrasyon Parametreleri (Ağırlık Matrisleri)
        # Q: Hataları cezalandırır. [Cross-track Error, Heading Error]
        self.Q = np.eye(2)
        self.Q[0, 0] = 1.0  # Yoldan sapma hatasına verilen önem
        self.Q[1, 1] = 0.5  # Açısal sapmaya verilen önem

        # R: Kontrol eforunu cezalandırır (Direksiyonu yumuşak tutmak için)
        self.R = np.eye(1)
        self.R[0, 0] = 0.1  # Direksiyonu kırma cezası

        self.dt = 0.1  # Simülasyon zaman adımı [s]
        self.WB = 2.5  # Aracın dingil mesafesi [m]

    def solve_dare(self, A, B, max_iter=150, eps=0.01):
        """
        Ayrık Zamanlı Cebirsel Riccati Denklemini (DARE) çözer.
        Optimal kontrol kazancını (K) bulmak için kullanılır.
        """
        P = self.Q
        for i in range(max_iter):
            # Riccati iterasyonu
            Pn = A.T @ P @ A - A.T @ P @ B @ np.linalg.inv(self.R + B.T @ P @ B) @ B.T @ P @ A + self.Q
            if abs(Pn - P).max() < eps:
                break
            P = Pn
            
        # Optimal Kazanç Matrisi (K)
        K = np.linalg.inv(self.R + B.T @ P @ B) @ (B.T @ P @ A)
        return K

    def lqr_steering_control(self, state, cx, cy, cyaw, target_ind):
        """
        LQR kullanarak optimal direksiyon açısını hesaplar.
        """
        # 1. Hata Durumlarını Hesapla (State Vector)
        # e: Cross-track error (Yoldan sapma mesafesi)
        # th_e: Heading error (Açısal hata)
        
        fx = state.x + state.WB * math.cos(state.yaw)
        fy = state.y + state.WB * math.sin(state.yaw)

        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_ind = np.argmin(d)

        # Hatanın yönünü bul (Sağda mı solda mı?)
        front_axle_vec = [-math.cos(state.yaw + math.pi / 2),
                          -math.sin(state.yaw + math.pi / 2)]
        e = np.dot([dx[target_ind], dy[target_ind]], front_axle_vec)
        
        th_e = self.normalize_angle(cyaw[target_ind] - state.yaw)

        # Durum Vektörü x = [e, th_e]^T
        x = np.array([[e], [th_e]])

        # 2. Lineer Kinematik Modeli Oluştur (State-Space)
        # x(k+1) = A * x(k) + B * u(k)
        v = state.v
        if v < 1e-2: v = 1e-2  # Sıfıra bölme hatasını önle

        A = np.zeros((2, 2))
        A[0, 0] = 1.0
        A[0, 1] = v * self.dt
        A[1, 0] = 0.0
        A[1, 1] = 1.0

        B = np.zeros((2, 1))
        B[0, 0] = 0.0
        B[1, 0] = (v / self.WB) * self.dt

        # 3. LQR Çözücü ile Optimal Kazancı Bul
        K = self.solve_dare(A, B)

        # 4. Kontrol Komutunu Uygula: u = -K * x
        u = -K @ x
        
        # Direksiyon açısını (delta) radyan cinsinden al
        delta = u[0, 0]
        
        # Fiziksel limitleri uygula (Maks 40 derece)
        delta = np.clip(delta, -math.radians(40), math.radians(40))

        return delta, target_ind, e

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle
