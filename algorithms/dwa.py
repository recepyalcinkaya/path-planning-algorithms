import math
import numpy as np

class DWAConfig:
    def __init__(self):
        # Elektrikli Araç (EV) Kinematik Limitleri
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s] (Geri vites)
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s] (Direksiyon dönüş hızı)
        self.max_accel = 0.2  # [m/ss] (Motor maksimum ivmelenmesi)
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        
        # Hesaplama Çözünürlüğü
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        
        self.dt = 0.1  # [s] (Simülasyon zaman adımı)
        self.predict_time = 3.0  # [s] (Aracın gelecekte nerede olacağını tahmin süresi)
        
        # Optimizasyon Ağırlıkları (Cost Function)
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        
        self.robot_radius = 1.0  # [m] (Aracın fiziksel boyutu / Hitbox)

class DWAPlanner:
    def __init__(self, config=DWAConfig()):
        self.config = config

    def motion(self, x, u, dt):
        # Aracın fiziksel hareketi (Kinematik Model)
        # x: [x, y, theta, v, omega] | u: [hız komutu, dönüş komutu]
        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]
        return x

    def calc_dynamic_window(self, x):
        # Araç motorunun ivme kısıtlarına göre ulaşılabilecek Hız/Dönüş penceresi
        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yaw_rate, self.config.max_yaw_rate]

        Vd = [x[3] - self.config.max_accel * self.config.dt,
              x[3] + self.config.max_accel * self.config.dt,
              x[4] - self.config.max_delta_yaw_rate * self.config.dt,
              x[4] + self.config.max_delta_yaw_rate * self.config.dt]

        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        return dw

    def predict_trajectory(self, x_init, v, y):
        # Seçilen hız ve açıya göre aracın rotasını tahmin et
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0.0
        while time <= self.config.predict_time:
            x = self.motion(x, [v, y], self.config.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.config.dt
        return trajectory

    def calc_control_and_trajectory(self, x, dw, goal, ob):
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # Dinamik pencere içindeki tüm hız ve açı kombinasyonlarını dene
        for v in np.arange(dw[0], dw[1], self.config.v_resolution):
            for y in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):
                trajectory = self.predict_trajectory(x_init, v, y)
                
                # Maliyetleri (Cost) hesapla
                to_goal_cost = self.calc_to_goal_cost(trajectory, goal)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - trajectory[-1, 3])
                ob_cost = self.calc_obstacle_cost(trajectory, ob)

                final_cost = to_goal_cost + speed_cost + ob_cost

                # En güvenli ve mantıklı yolu seç
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    
        return best_u, best_trajectory

    def calc_obstacle_cost(self, trajectory, ob):
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if np.array(r <= self.config.robot_radius).any():
            return float("Inf") # Çarpışma durumu, maliyet sonsuz

        min_r = np.min(r)
        return 1.0 / min_r  # Engellere uzak olmayı ödüllendir

    def calc_to_goal_cost(self, trajectory, goal):
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        return self.config.to_goal_cost_gain * cost

    def planning(self, x, goal, ob):
        dw = self.calc_dynamic_window(x)
        u, trajectory = self.calc_control_and_trajectory(x, dw, goal, ob)
        return u, trajectory
