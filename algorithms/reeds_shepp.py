import math
import numpy as np

class ReedsSheppPlanner:
    def __init__(self, turning_radius=5.0, step_size=0.1):
        self.turning_radius = turning_radius
        self.step_size = step_size

    def planning(self, sx, sy, syaw, ex, ey, eyaw):
        # Koordinatları normalize et (Başlangıç noktasını 0,0 varsay)
        dx = ex - sx
        dy = ey - sy
        cos_theta = math.cos(syaw)
        sin_theta = math.sin(syaw)
        
        lex = (cos_theta * dx + sin_theta * dy) / self.turning_radius
        ley = (-sin_theta * dx + cos_theta * dy) / self.turning_radius
        leyaw = eyaw - syaw

        # En kısa Reeds-Shepp yolunu hesapla
        best_path = self.get_best_path(lex, ley, leyaw)
        
        if not best_path:
            return [], [], [], []

        # Yol noktalarını oluştur
        px, py, pyaw, directions = self.generate_course(best_path)
        
        # Gerçek dünya koordinatlarına geri dönüştür
        rx, ry, ryaw = [], [], []
        for ix, iy, iyaw in zip(px, py, pyaw):
            ix *= self.turning_radius
            iy *= self.turning_radius
            
            converted_x = math.cos(-syaw) * ix + math.sin(-syaw) * iy + sx
            converted_y = -math.sin(-syaw) * ix + math.cos(-syaw) * iy + sy
            converted_yaw = (iyaw + syaw) % (2.0 * math.pi)
            
            rx.append(converted_x)
            ry.append(converted_y)
            ryaw.append(converted_yaw)

        return rx, ry, ryaw, directions

    def get_best_path(self, x, y, phi):
        # Basitleştirilmiş Reeds-Shepp seçici (LSL, LSR, RSR, RSL manevraları ve tersleri)
        # Profesyonel kütüphanelerde 48 farklı durum kontrol edilir
        best_path = {"lengths": [float("inf")], "types": []}
        
        # Örnek: Basit manevra tipleri testi
        # Bu fonksiyon normalde tüm optimal kombinasyonları tarar
        # Burada temel bir manevra seti simüle edilmiştir
        paths = self.generate_all_paths(x, y, phi)
        
        min_len = float("inf")
        selected_path = None
        for path in paths:
            length = sum([abs(l) for l in path["lengths"]])
            if length < min_len:
                min_len = length
                selected_path = path
        
        return selected_path

    def generate_all_paths(self, x, y, phi):
        # Bu alan algoritmanın 'beyni'dir; tüm geometrik formülleri içerir
        # Örnek amaçlı birkaç temel Reeds-Shepp kombinasyonu:
        return [
            {"types": ["L", "S", "L"], "lengths": [0.5, 1.0, 0.5]},
            {"types": ["R", "S", "L"], "lengths": [-0.3, 0.8, 0.4]}, # Eksi değer geri gitmeyi temsil eder
            {"types": ["L", "R", "L"], "lengths": [0.6, -0.4, 0.6]}
        ]

    def generate_course(self, path):
        px, py, pyaw, directions = [0.0], [0.0], [0.0], []
        
        for t, l in zip(path["types"], path["lengths"]):
            step = self.step_size if l > 0 else -self.step_size
            d = 0.0
            while abs(d) < abs(l):
                px.append(px[-1] + step * math.cos(pyaw[-1]))
                py.append(py[-1] + step * math.sin(pyaw[-1]))
                
                if t == "L": pyaw.append(pyaw[-1] + step)
                elif t == "R": pyaw.append(pyaw[-1] - step)
                else: pyaw.append(pyaw[-1])
                
                directions.append(1 if l > 0 else -1)
                d += step
                
        return px, py, pyaw, directions
