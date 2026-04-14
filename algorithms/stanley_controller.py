import numpy as np
import math

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.WB = 2.5 # Wheelbase [m]

    def update(self, accel, delta, dt):
        # Aracın kinematik bisiklet modeli
        delta = np.clip(delta, -0.6, 0.6) # Direksiyon kısıtı

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / self.WB * math.tan(delta) * dt
        self.v += accel * dt

class StanleyController:
    def __init__(self):
        self.k = 0.5      # Kontrol kazancı (Gain)
        self.Kp = 1.0     # Hız kontrolü (P gain)
        self.dt = 0.1     # Zaman adımı [s]

    def pid_control(self, target_v, current_v):
        return self.Kp * (target_v - current_v)

    def stanley_control(self, state, cx, cy, cyaw, last_id):
        # 1. En yakın yolu ve Cross-track hatasını (e) bul
        current_id, error_front_axle = self.calc_target_index(state, cx, cy)

        if last_id >= current_id:
            current_id = last_id

        # 2. Heading error (theta_e): Yolun açısı ile aracın açısı arasındaki fark
        theta_e = self.normalize_angle(cyaw[current_id] - state.yaw)
        
        # 3. Cross-track steering (delta_e): Yolun dışındaysak yola döndüren açı
        # Formül: atan(k * e / v)
        theta_d = math.atan2(self.k * error_front_axle, state.v + 1e-6) # Sıfıra bölme hatası için 1e-6

        # Toplam direksiyon açısı (delta)
        delta = theta_e + theta_d

        return delta, current_id

    def calc_target_index(self, state, cx, cy):
        # Ön aksın (Front Axle) merkezini hesapla
        fx = state.x + state.WB * math.cos(state.yaw)
        fy = state.y + state.WB * math.sin(state.yaw)

        # Ön aksa en yakın yol noktasını bul
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Cross-track hatasının (e) yönünü belirle (Yolun sağında mı solunda mı?)
        front_axle_vec = [-math.cos(state.yaw + math.pi / 2),
                          -math.sin(state.yaw + math.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle
