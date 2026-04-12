import matplotlib.pyplot as plt
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.potential_field import PotentialFieldPlanner

def main():
    print(__file__ + " Otonom Araç APF Simülasyonu Başlatıldı!!")

    start_x, start_y = 0.0, 10.0
    goal_x, goal_y = 50.0, 50.0
    grid_size = 0.5

    # Engeller (Birkaç küme halinde koyalım ki etki alanları görünsün)
    obstacle_x, obstacle_y = [], []
    for i in range(15, 25):
        obstacle_x.append(i); obstacle_y.append(25.0)
    for i in range(10, 30):
        obstacle_x.append(30.0); obstacle_y.append(i)
    for i in range(35, 45):
        obstacle_x.append(i); obstacle_y.append(40.0)

    # APF Algoritmasını Başlat
    planner = PotentialFieldPlanner(obstacle_x, obstacle_y, grid_size)
    
    # Görselleştirme için Isı Haritası (Heatmap) verisini al
    print("Isı Haritası (Heatmap) hesaplanıyor, lütfen bekleyin...")
    x_range, y_range, pmap = planner.generate_heatmap(-10, 60, -10, 60)

    # Yolu Planla
    rx, ry = planner.planning(start_x, start_y, goal_x, goal_y)

    # --- Çizim (Heatmap ve Rota) ---
    plt.figure(figsize=(10, 8))
    
    # 1. Isı Haritasını Çiz (pcolor)
    X, Y = np.meshgrid(x_range, y_range)
    # Değerleri logaritmik olarak sınırla ki renkler çok patlamasın
    pmap_clipped = np.clip(pmap.T, 0.0, 100.0) 
    plt.pcolor(X, Y, pmap_clipped, cmap='jet', vmax=100.0, shading='auto')
    plt.colorbar(label='Potansiyel Enerji Seviyesi')

    # 2. Engelleri, Başlangıcı ve Hedefi Çiz
    plt.plot(obstacle_x, obstacle_y, "*k", label="Engeller")
    plt.plot(start_x, start_y, "or", markersize=10, label="Başlangıç")
    plt.plot(goal_x, goal_y, "*m", markersize=15, label="Hedef")

    # 3. Hesaplanan Yolu Çiz
    if rx and ry:
        plt.plot(rx, ry, "-w", linewidth=3, label="APF Rotası")
        
    plt.title("Yapay Potansiyel Alanlar (APF) - Isı Haritası")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.axis("equal")
    plt.grid(False) # Heatmap olduğu için gridi kapatıyoruz
    plt.show()

if __name__ == '__main__':
    main()
