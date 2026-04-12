import numpy as np
import math

class PotentialFieldPlanner:
    def __init__(self, obstacle_x, obstacle_y, grid_size=0.5, robot_radius=1.0):
        # Parametreler (Ar-Ge ekiplerinin ince ayar yaptığı yerler)
        self.KP = 5.0  # Çekici kuvvet (Attractive potential) katsayısı
        self.ETA = 100.0  # İtici kuvvet (Repulsive potential) katsayısı
        self.AREA_WIDTH = 30.0  # Engellerin itme kuvvetinin etki alanı [m]
        
        self.grid_size = grid_size
        self.robot_radius = robot_radius
        
        self.ox = obstacle_x
        self.oy = obstacle_y

    def calc_attractive_potential(self, x, y, gx, gy):
        # Hedefe olan mesafe (Ne kadar uzaksa o kadar güçlü çeker)
        return 0.5 * self.KP * np.hypot(x - gx, y - gy)

    def calc_repulsive_potential(self, x, y):
        # Tüm engellere olan itme kuvvetini hesapla
        min_id = -1
        dmin = float("inf")
        for i, _ in enumerate(self.ox):
            d = np.hypot(x - self.ox[i], y - self.oy[i])
            if dmin >= d:
                dmin = d
                min_id = i

        # Engelin etki alanının dışındaysa itme kuvveti sıfırdır
        dq = np.hypot(x - self.ox[min_id], y - self.oy[min_id])
        if dq <= self.AREA_WIDTH:
            if dq <= 0.1:
                dq = 0.1
            return 0.5 * self.ETA * (1.0 / dq - 1.0 / self.AREA_WIDTH) ** 2
        else:
            return 0.0

    def calc_total_potential(self, x, y, gx, gy):
        # Toplam potansiyel (Çekme + İtme)
        return self.calc_attractive_potential(x, y, gx, gy) + self.calc_repulsive_potential(x, y)

    def planning(self, sx, sy, gx, gy):
        print("Potential Field Planlaması Başladı...")
        
        # Simülasyon izi için
        rx, ry = [sx], [sy]
        
        # Gradient Descent (Bayır Aşağı İniş) algoritması
        d = np.hypot(sx - gx, sy - gy)
        ix = sx
        iy = sy
        
        # Maksimum iterasyon (Sonsuz döngüyü engellemek için)
        max_iters = 1000
        iters = 0
        
        while d >= self.grid_size and iters < max_iters:
            # X yönündeki kuvvet (Kısmi türev)
            vx = self.calc_total_potential(ix + self.grid_size, iy, gx, gy) - \
                 self.calc_total_potential(ix, iy, gx, gy)
                 
            # Y yönündeki kuvvet (Kısmi türev)
            vy = self.calc_total_potential(ix, iy + self.grid_size, gx, gy) - \
                 self.calc_total_potential(ix, iy, gx, gy)

            # Kuvvetin büyüklüğünü hesapla
            v = np.hypot(vx, vy)
            
            # Aracı normalize edilmiş kuvvet yönünde bir adım kaydır
            ix -= self.grid_size * (vx / v)
            iy -= self.grid_size * (vy / v)

            rx.append(ix)
            ry.append(iy)
            
            d = np.hypot(ix - gx, iy - gy)
            iters += 1
            
        if iters >= max_iters:
            print("Local Minima'ya takıldı! (Araç iki engel ve hedef arasında sıkıştı)")
            
        return rx, ry

    def generate_heatmap(self, min_x, max_x, min_y, max_y):
        # Tüm haritanın potansiyel enerji tablosunu oluştur (Görselleştirme için)
        x_range = np.arange(min_x, max_x, self.grid_size)
        y_range = np.arange(min_y, max_y, self.grid_size)
        
        pmap = np.zeros((len(x_range), len(y_range)))
        
        # Sadece görselleştirme amaçlı hedef noktası (haritanın en ucu)
        gx, gy = max_x - 10, max_y - 10 
        
        for ix, x in enumerate(x_range):
            for iy, y in enumerate(y_range):
                pmap[ix, iy] = self.calc_total_potential(x, y, gx, gy)
                
        return x_range, y_range, pmap
