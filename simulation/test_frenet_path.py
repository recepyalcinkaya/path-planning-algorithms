import matplotlib.pyplot as plt
import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.frenet_optimal_trajectory import frenet_optimal_planning

def main():
    # Başlangıç Durumu (Frenet)
    s0 = 0.0        # Başlangıç konumu
    s0_d = 10.0/3.6 # Başlangıç hızı
    s0_dd = 0.0     # Başlangıç ivmesi
    l0 = 2.0        # Şeridin solunda (2m offset)
    l0_d = 0.0
    l0_dd = 0.0
    
    target_speed = 30.0 / 3.6 # 30 km/h hedef

    paths = frenet_optimal_planning(s0, s0_d, s0_dd, l0, l0_d, l0_dd, target_speed)

    # En iyi yolu bul (Minimum cost)
    best_path = min(paths, key=lambda p: p.cost)

    # Çizim
    plt.figure(figsize=(10, 6))
    
    # Tüm aday yörüngeleri gri çiz
    for path in paths:
        plt.plot(path.s, path.d, "-g", alpha=0.1)

    # En iyi yörüngeyi mavi çiz
    plt.plot(best_path.s, best_path.d, "-b", linewidth=3, label="Optimal Path")
    
    # Yol sınırlarını çiz
    plt.axhline(y=3.5, color='k', linestyle='--')
    plt.axhline(y=-3.5, color='k', linestyle='--')
    plt.fill_between([0, 60], 3.5, -3.5, color='gray', alpha=0.2)

    plt.title("Ar-Ge Seviyesi Frenet Optimal Trajectory Planlaması")
    plt.xlabel("S (Yol Boyunca İlerleme) [m]")
    plt.ylabel("L (Şerit Offseti) [m]")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    main()
