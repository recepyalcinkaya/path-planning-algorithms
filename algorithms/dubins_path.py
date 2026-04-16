import math
import numpy as np

class DubinsPathPlanner:
    def __init__(self, turning_radius=5.0, step_size=0.1):
        self.turning_radius = turning_radius  # Aracın minimum dönüş yarıçapı [m]
        self.step_size = step_size            # Çizim hassasiyeti [m]

    def planning(self, sx, sy, syaw, ex, ey, eyaw):
        """
        Başlangıç (s) ve Bitiş (e) noktaları arasında Dubins yolunu hesaplar.
        """
        # Hedefin başlangıca göre göreceli konumunu hesapla
        ex_local = ex - sx
        ey_local = ey - sy
        
        # Koordinat sistemini başlangıç açısına (syaw) göre döndür
        lex = math.cos(syaw) * ex_local + math.sin(syaw) * ey_local
        ley = -math.sin(syaw) * ex_local + math.cos(syaw) * ey_local
        leyaw = eyaw - syaw

        # Yarıçapa bölerek normalize et
        D = math.hypot(lex, ley) / self.turning_radius
        d = D
        alpha = math.atan2(ley, lex) % (2.0 * math.pi)
        beta = (leyaw - alpha) % (2.0 * math.pi)

        # Tüm olası yörünge tiplerini (LSL, RSR, LSR, vb.) test et
        planners = [self.LSL, self.RSR, self.LSR, self.RSL]
        best_cost = float("inf")
        best_mode = None
        best_lengths = None

        for planner in planners:
            mode, lengths = planner(alpha, beta, d)
            if mode:
                cost = sum([abs(l) for l in lengths])
                if best_cost > cost:
                    best_cost = cost
                    best_mode = mode
                    best_lengths = lengths

        if best_mode is None:
            print("Uygun bir Dubins yolu bulunamadı!")
            return [], [], []

        # En iyi yolu bulduktan sonra noktaları (X, Y, Yaw) oluştur
        px, py, pyaw = self.generate_course(best_lengths, best_mode)
        
        # Normalize edilmiş noktaları gerçek dünya koordinatlarına geri çevir
        rx, ry, ryaw = [], [], []
        for ix, iy, iyaw in zip(px, py, pyaw):
            # Yeniden ölçeklendir ve döndür
            ix *= self.turning_radius
            iy *= self.turning_radius
            
            converted_x = math.cos(-syaw) * ix + math.sin(-syaw) * iy + sx
            converted_y = -math.sin(-syaw) * ix + math.cos(-syaw) * iy + sy
            converted_yaw = (iyaw + syaw) % (2.0 * math.pi)
            
            rx.append(converted_x)
            ry.append(converted_y)
            ryaw.append(converted_yaw)

        return rx, ry, ryaw, best_mode

    # --- Matematiksel Geometri Fonksiyonları (LSL, RSR, vs.) ---
    def LSL(self, alpha, beta, d):
        tmp0 = d + math.sin(alpha) - math.sin(beta)
        p_squared = 2 + d**2 - 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) - math.sin(beta))
        if p_squared < 0: return None, None
        p = math.sqrt(p_squared)
        tmp2 = math.atan2(math.cos(beta) - math.cos(alpha), tmp0)
        t = (tmp2 - alpha) % (2 * math.pi)
        q = (beta - tmp2) % (2 * math.pi)
        return ["L", "S", "L"], [t, p, q]

    def RSR(self, alpha, beta, d):
        tmp0 = d - math.sin(alpha) + math.sin(beta)
        p_squared = 2 + d**2 - 2 * math.cos(alpha - beta) + 2 * d * (math.sin(beta) - math.sin(alpha))
        if p_squared < 0: return None, None
        p = math.sqrt(p_squared)
        tmp2 = math.atan2(math.cos(alpha) - math.cos(beta), tmp0)
        t = (alpha - tmp2) % (2 * math.pi)
        q = (tmp2 - beta) % (2 * math.pi)
        return ["R", "S", "R"], [t, p, q]

    def LSR(self, alpha, beta, d):
        p_squared = -2 + d**2 + 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) + math.sin(beta))
        if p_squared < 0: return None, None
        p = math.sqrt(p_squared)
        tmp2 = math.atan2(-math.cos(alpha) - math.cos(beta), d + math.sin(alpha) + math.sin(beta))
        t = (tmp2 - alpha) % (2 * math.pi)
        q = (tmp2 - beta) % (2 * math.pi)
        return ["L", "S", "R"], [t, p, q]

    def RSL(self, alpha, beta, d):
        p_squared = -2 + d**2 + 2 * math.cos(alpha - beta) - 2 * d * (math.sin(alpha) + math.sin(beta))
        if p_squared < 0: return None, None
        p = math.sqrt(p_squared)
        tmp2 = math.atan2(math.cos(alpha) + math.cos(beta), d - math.sin(alpha) - math.sin(beta))
        t = (alpha - tmp2) % (2 * math.pi)
        q = (beta - tmp2) % (2 * math.pi)
        return ["R", "S", "L"], [t, p, q]

    def generate_course(self, lengths, mode):
        px, py, pyaw = [0.0], [0.0], [0.0]
        
        for m, l in zip(mode, lengths):
            pd = 0.0
            while pd < abs(l):
                px.append(px[-1] + self.step_size * math.cos(pyaw[-1]))
                py.append(py[-1] + self.step_size * math.sin(pyaw[-1]))
                
                if m == "L":   pyaw.append(pyaw[-1] + self.step_size)
                elif m == "R": pyaw.append(pyaw[-1] - self.step_size)
                elif m == "S": pyaw.append(pyaw[-1])
                
                pd += self.step_size
                
        return px, py, pyaw
