import matplotlib.pyplot as plt
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.rrt import RRTPlanner

def main():
    print(__file__ + " start!!")

    start_x, start_y = 10.0, 10.0
    goal_x, goal_y = 50.0, 50.0
    robot_radius = 2.0 
    
    # RRT'nin rastgele nokta atacağı harita sınırları (A* testindeki alan)
    rand_area = [-10.0, 60.0]

    # Engelleri belirle (A* ve Dijkstra testindeki ile birebir aynı harita)
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

    # Matplotlib Plot ayarları (Animasyonlu çizim için)
    plt.plot(obstacle_x, obstacle_y, ".k")
    plt.plot(start_x, start_y, "og") 
    plt.plot(goal_x, goal_y, "xb")
    plt.grid(True)
    plt.axis("equal")

    # RRT Algoritmasını Çalıştır
    rrt = RRTPlanner(obstacle_x, obstacle_y, rand_area, expand_dis=4.0, goal_sample_rate=10, robot_radius=robot_radius)
    
    path = rrt.planning(start_x, start_y, goal_x, goal_y, animation=True)

    # Oluşan Ağacı Çizdir (Opsiyonel ama çok havalı durur)
    for node in rrt.node_list:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g", alpha=0.3)

    # Nihai Yolu Çizdir
    if path is None:
        print("Hedefe ulaşılamadı!")
    else:
        path_x = [x for (x, y) in path]
        path_y = [y for (x, y) in path]
        plt.plot(path_x, path_y, '-r', linewidth=2) # r = red line (bulunan yol)

    plt.pause(0.001)
    plt.show()

if __name__ == '__main__':
    main()
