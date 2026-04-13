import math
import numpy as np

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.WB = 2.5  # Wheelbase: Aracın ön ve arka tekerlekleri arasındaki mesafe [m]

    def update(self, a, delta, dt):
        # Aracın konumunu güncelle (Kinematik Bisiklet Modeli)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / self.WB * math.tan(delta) * dt
        self.v += a * dt

class PurePursuitController:
    def __init__(self):
        # Ar-Ge Kalibrasyon Parametreleri
        self.k = 0.1       # Hıza bağlı lookahead (ileri bakma) çarpanı
        self.Lfc = 2.0     # Minimum lookahead mesafesi [m]
        self.Kp = 1.0      # Hız kontrolü için oransal kazanç (PID'nin P'si)
        self.dt = 0.1      # Zaman adımı [s]
        
    def proportional_control(self, target_v, current_v):
        # Basit ivme kontrolü
        a = self.Kp * (target_v - current_v)
        return a

    def calc_target_index(self, state, cx, cy):
        # Araca en yakın yol noktasını bul
        dx = [state.x - icx for icx in cx]
        dy = [state.y - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        # Lookahead (Ld) mesafesini aracın anlık hızına göre dinamik olarak hesapla
        # Araç ne kadar hızlıysa, o kadar uzağa bakmalıdır!
        Lf = self.k * state.v + self.Lfc

        # Lookahead çemberinin yolu kestiği noktayı bul (İleriye doğru)
        while Lf > state.calc_distance(cx[target_idx], cy[target_idx]):
            if (target_idx + 1) >= len(cx):
                break  # Yolun sonuna gelindi
            target_idx += 1

        return target_idx, Lf

    def pure_pursuit_steer_control(self, state, trajectory_x, trajectory_y, target_idx):
        # Hedef noktanın koordinatları
        tx = trajectory_x[target_idx]
        ty = trajectory_y[target_idx]

        # Aracın arka aksından hedef noktaya olan açıyı (alpha) hesapla
        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

        # Hedef noktaya olan gerçek mesafe (Lookahead)
        Lf = math.hypot(tx - state.x, ty - state.y)

        # Geometrik Pure Pursuit direksiyon açısı formülü:
        # delta = atan2(2 * L * sin(alpha) / Ld)
        delta = math.atan2(2.0 * state.WB * math.sin(alpha) / Lf, 1.0)

        return delta, target_idx

    def calc_distance(self, state, point_x, point_y):
        dx = state.x - point_x
        dy = state.y - point_y
        return math.hypot(dx, dy)

# Helper function injection to the State class for cleaner code
State.calc_distance = PurePursuitController.calc_distance
