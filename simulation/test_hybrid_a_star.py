import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.hybrid_a_star import HybridAStarPlanner

def main():
    print(__file__ + " Otonom Araç Hybrid A* Simülasyonu Başlatıldı!!")

    # Başlangıç: [x, y, yaw(radyan)]
    start_x, start_y, start_yaw = 10.0, 10.0, math.pi / 2.0  # Kuzeye bakıyor
    # Hedef: [x, y, yaw(radyan)]
    goal_x, goal_y, goal_yaw = 50.0, 50.0, 0.0             # Doğuya bakıyor

    # Engelleri belirle (Ortak haritamız)
    obstacle_x, obstacle_y = [], []
    for i in range(-10, 60):
        obstacle_x.append(i); obstacle_y.append(-10.0)
        obstacle_x.append(60.0); obstacle_y.append(i)
        obstacle_x.append(i); obstacle_y.append(60.0)
        obstacle_x.append(-10.0); obstacle_y.append(i)
    for i in range(-10, 40):
        obstacle_x.append(20.0); obstacle_y.append(i)
    for i in range(0, 40):
        obstacle_x.append(40.0); obstacle_y.append(60.0 - i)

    # Matplotlib Plot ayarları
    plt.plot(obstacle_x, obstacle_y, ".k")
    plt.plot(start_x, start_y, "og") 
    plt.plot(goal_x, goal_y, "xb")
    
    # Başlangıç ve hedef yönünü çiz (Ok işareti ile)
    plt.quiver(start_x, start_y, math.cos(start_yaw), math.sin(start_yaw), color='g', scale=15)
    plt.quiver(goal_x, goal_y, math.cos(goal_yaw), math.sin(goal_yaw), color='b', scale=15)
    
    plt.grid(True)
    plt.axis("equal")

    # Hybrid A* Algoritmasını Çalıştır
    planner = HybridAStarPlanner(obstacle_x, obstacle_y)
    rx, ry, ryaw = planner.planning(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw)

    # Sonucu Çizdir
    if not rx:
        print("Hedefe ulaşılamadı!")
    else:
        # Pürüzsüz kavisli yolu çiz (Kırmızı çizgi)
        plt.plot(rx, ry, "-r", linewidth=2)
        
        # Belirli aralıklarla aracın yönelimini (yaw) oklarla göster
        for i in range(0, len(rx), 2):
            plt.quiver(rx[i], ry[i], math.cos(ryaw[i]), math.sin(ryaw[i]), color='r', alpha=0.5, scale=20)
            plt.pause(0.01)

    plt.title("Hybrid A* Path Planning (Kinematic Curve)")
    plt.show()

if __name__ == '__main__':
    main()
