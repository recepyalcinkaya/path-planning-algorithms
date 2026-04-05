import matplotlib.pyplot as plt
import sys
import os

# algorithms klasörüne erişim sağlamak için path ekliyoruz
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.dijkstra import DijkstraPlanner

def main():
    print(__file__ + " start!!")

    # Başlangıç ve Bitiş noktaları
    start_x, start_y = 10.0, 10.0
    goal_x, goal_y = 50.0, 50.0
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # Engelleri belirle (A* testindeki ile birebir aynı harita)
    obstacle_x, obstacle_y = [], []
    
    # Dış Duvarlar
    for i in range(-10, 60):
        obstacle_x.append(i)
        obstacle_y.append(-10.0)
        obstacle_x.append(60.0)
        obstacle_y.append(i)
        obstacle_x.append(i)
        obstacle_y.append(60.0)
        obstacle_x.append(-10.0)
        obstacle_y.append(i)

    # Ortadaki Engeller
    for i in range(-10, 40):
        obstacle_x.append(20.0)
        obstacle_y.append(i)
    for i in range(0, 40):
        obstacle_x.append(40.0)
        obstacle_y.append(60.0 - i)

    # Matplotlib Plot ayarları
    plt.plot(obstacle_x, obstacle_y, ".k") # k = black dots (engeller)
    plt.plot(start_x, start_y, "og")       # g = green circle (başlangıç)
    plt.plot(goal_x, goal_y, "xb")         # b = blue cross (hedef)
    plt.grid(True)
    plt.axis("equal")

    # Dijkstra Algoritmasını Çalıştır
    dijkstra = DijkstraPlanner(obstacle_x, obstacle_y, grid_size, robot_radius)
    rx, ry = dijkstra.planning(start_x, start_y, goal_x, goal_y)

    # Sonucu Çizdir
    plt.plot(rx, ry, "-r") # r = red line (bulunan yol)
    plt.pause(0.001)
    plt.show()

if __name__ == '__main__':
    main()
