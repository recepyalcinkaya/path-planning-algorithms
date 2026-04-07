import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import os

# Kütüphane yolunu ekle
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.dwa import DWAPlanner

def main():
    print(__file__ + " Otonom Araç DWA Simülasyonu Başlatıldı!!")
    
    # Başlangıç durumu [x(m), y(m), yaw(radyan), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    goal = np.array([10.0, 10.0])
    
    # Sensörlerden gelen anlık dinamik engeller [x(m), y(m)]
    ob = np.array([[-1, -1], [0, 2], [4.0, 2.0], [5.0, 4.0], 
                   [5.0, 5.0], [5.0, 6.0], [5.0, 9.0], [8.0, 9.0]])

    planner = DWAPlanner()
    trajectory = np.array(x)
    
    while True:
        # 1. DWA ile anlık en iyi Hız ve Direksiyon komutunu (u) bul
        u, predicted_trajectory = planner.planning(x, goal, ob)
        
        # 2. Aracı bir adım ilerlet
        x = planner.motion(x, u, planner.config.dt)
        trajectory = np.vstack((trajectory, x))

        # --- Animasyon Çizimi ---
        plt.cla()
        # Arabanın tahmin ettiği rotayı çiz (Yeşil)
        plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
        # Arabanın kendisini çiz (Kırmızı)
        plt.plot(x[0], x[1], "xr")
        # Hedefi çiz (Mavi)
        plt.plot(goal[0], goal[1], "xb")
        # Engelleri çiz (Siyah Noktalar)
        plt.plot(ob[:, 0], ob[:, 1], "ok")
        # Aracın bıraktığı iz (Kırmızı Çizgi)
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        
        plt.axis("equal")
        plt.grid(True)
        plt.title(f"DWA Local Planning - Hız: {x[3]:.2f} m/s")
        plt.pause(0.001)

        # Hedefe varış kontrolü
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= planner.config.robot_radius:
            print("Hedefe Ulaşıldı!")
            break

    plt.show()

if __name__ == '__main__':
    main()
