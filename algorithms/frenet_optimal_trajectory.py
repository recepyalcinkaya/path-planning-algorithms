import numpy as np
import copy
import math

class QuinticPolynomial:
    def __init__(self, xs, v_s, a_s, xe, v_e, a_e, time):
        # 5. Dereceden Polinom Katsayıları (Minimum Jerk Trajectory)
        self.a0 = xs
        self.a1 = v_s
        self.a2 = a_s / 2.0

        A = np.array([[time**3, time**4, time**5],
                      [3 * time**2, 4 * time**3, 5 * time**4],
                      [6 * time, 12 * time**2, 20 * time**3]])
        B = np.array([xe - self.a0 - self.a1 * time - self.a2 * time**2,
                      v_e - self.a1 - 2 * self.a2 * time,
                      a_e - 2 * self.a2])
        x = np.linalg.solve(A, B)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
             self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5
        return xt

class FrenetPath:
    def __init__(self):
        self.t, self.d, self.d_d, self.d_dd, self.d_ddd = [], [], [], [], []
        self.s, self.s_d, self.s_dd, self.s_ddd = [], [], [], []
        self.x, self.y, self.yaw, self.ds, self.c = [], [], [], [], []
        self.cost = 0.0

def frenet_optimal_planning(si, si_d, si_dd, li, li_d, li_dd, target_speed):
    # Ar-Ge ekiplerinin parametre havuzu
    MAX_SPEED = 50.0 / 3.6  # m/s
    MAX_ACCEL = 2.0         # m/s^2
    MAX_CURVATURE = 1.0     # 1/m
    ROAD_WIDTH = 7.0        # m
    D_ROAD_W = 1.0          # m
    DT = 0.2                # s
    MAX_T = 5.0             # Planlama ufku (s)
    MIN_T = 4.0             # Planlama ufku (s)
    TARGET_SPEED = target_speed 
    
    # Maliyet Ağırlıkları
    K_J = 0.1; K_T = 0.1; K_D = 1.0; K_LAT = 1.0; K_LON = 1.0

    frenet_paths = []

    # Lateral (Yan) hareket için farklı hedefleri tara (Şerit değiştirme denemeleri)
    for di in np.arange(-ROAD_WIDTH, ROAD_WIDTH, D_ROAD_W):
        for Ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()
            lat_qp = QuinticPolynomial(li, li_d, li_dd, di, 0.0, 0.0, Ti)
            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            
            # Boylamsal (Hız) planlaması - Sabit hız senaryosu
            for tv in np.arange(TARGET_SPEED - 0.5, TARGET_SPEED + 0.5, 0.2):
                tfp = copy.deepcopy(fp)
                lon_qp = QuinticPolynomial(si, si_d, si_dd, si + tv * Ti, tv, 0.0, Ti)
                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                
                # Maliyet Hesaplama (Jerk + Zaman + Hedef Uzaklığı)
                # Profesyonel Ar-Ge bu 'cost' fonksiyonuyla oynayarak sürüş karakterini belirler
                tfp.cost = K_LAT * (sum([d**2 for d in tfp.d])) + \
                           K_LON * (sum([(s_d - TARGET_SPEED)**2 for s_d in [tv]*len(tfp.t)]))
                
                frenet_paths.append(tfp)

    return frenet_paths
