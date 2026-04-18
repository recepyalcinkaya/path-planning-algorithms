import matplotlib.pyplot as plt
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.reeds_shepp import ReedsSheppPlanner

def main():
    print("Reeds-Shepp (İleri ve Geri Manevra) Simülasyonu Başlatıldı!!")

    # Başlangıç ve Hedef (Paralel Park Senaryosu)
    start_x, start_y, start_yaw = 0.0, 0.0, 0.0
    goal_x, goal_y, goal_yaw = 10.0, -2.0, 0.0

    planner = ReedsSheppPlanner(turning_radius=3.0)
    rx, ry, ryaw, directions = planner.planning(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw)

    plt.figure(figsize=(10, 6))
    
    # Animasyon döngüsü
    for i in range(0, len(rx), 5):
        plt.cla()
        plt.plot(rx, ry, "--k", alpha=0.5) # Rotanın tamamı
        
        # Arabanın o anki konumu
        plt.plot(rx[i], ry[i], "or", markersize=10)
        
        # Yön oku (Mavi: İleri, Yeşil: Geri)
        color = "blue" if directions[i] > 0 else "green"
        plt.arrow(rx[i], ry[i], math.cos(ryaw[i])*1.5, math.sin(ryaw[i])*1.5, color=color, head_width=0.5)
        
        plt.title(f"Manevra Yönü: {'İLERİ' if directions[i]>0 else 'GERİ'}")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.01)

    plt.show()

if __name__ == '__main__':
    main()
