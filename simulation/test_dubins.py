import matplotlib.pyplot as plt
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.dubins_path import DubinsPathPlanner

def plot_arrow(x, y, yaw, length=2.0, width=0.5, fc="r", ec="k"):
    # Aracın yönünü gösteren şık bir ok çizimi
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_width=width, head_length=width, fc=fc, ec=ec)

def main():
    print(__file__ + " Dubins Eğrileri Simülasyonu Başlatıldı!!")

    # Senaryo 1: Paralel Park / Şerit Değiştirme tarzı bir manevra
    start_x = 0.0
    start_y = 0.0
    start_yaw = math.radians(45.0)  # Çapraz bakıyor

    goal_x = 15.0
    goal_y = -5.0
    goal_yaw = math.radians(-45.0)  # Aşağı doğru çapraz bakıyor

    turning_radius = 3.0 # Arabanın dönüş kabiliyeti [m]

    planner = DubinsPathPlanner(turning_radius=turning_radius)
    rx, ry, ryaw, mode = planner.planning(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw)

    if not rx:
        sys.exit()

    # --- Animasyon ---
    plt.figure(figsize=(10, 6))
    
    # Rota boyunca arabanın yönünü (okları) adım adım çizdir
    for i in range(0, len(rx), 10):
        plt.cla()
        
        # Tamamlanan rotayı çiz
        plt.plot(rx, ry, "-b", label=f"Dubins Yolu (Mod: {''.join(mode)})")
        
        # Başlangıç ve Bitiş noktaları
        plot_arrow(start_x, start_y, start_yaw, fc="g")
        plot_arrow(goal_x, goal_y, goal_yaw, fc="m")
        plt.plot(start_x, start_y, "og", label="Başlangıç")
        plt.plot(goal_x, goal_y, "om", label="Hedef")
        
        # Arabanın anlık konumu ve yönü
        plot_arrow(rx[i], ry[i], ryaw[i], fc="r")
        
        plt.axis("equal")
        plt.grid(True)
        plt.title(f"Dubins Path Planning - Manevra: {''.join(mode)}")
        plt.legend()
        plt.pause(0.01)

    plt.show()

if __name__ == '__main__':
    main()
