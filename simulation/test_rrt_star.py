import matplotlib.pyplot as plt
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.rrt_star import RRTStarPlanner

def main():
    print(__file__ + " RRT* Simülasyonu Başlatıldı!!")

    start_x, start_y = 10.0, 10.0
    goal_x, goal_y = 50.0, 50.0
    robot_radius = 2.0 
    rand_area = [-10.0, 60.0]

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

    plt.plot(obstacle_x, obstacle_y, ".k")
    plt.plot(start_x, start_y, "og") 
    plt.plot(goal_x, goal_y, "xb")
    plt.grid(True)
    plt.axis("equal")

    # RRT* Algoritmasını Çalıştır
    # max_iter değerini yüksek tuttuk ki optimizasyon için vakti olsun
    rrt_star = RRTStarPlanner(obstacle_x, obstacle_y, rand_area, expand_dis=4.0, 
                              max_iter=300, robot_radius=robot_radius)
    
    path = rrt_star.planning(start_x, start_y, goal_x, goal_y, animation=True)

    # Optimizasyon sonrası oluşan Rewired Ağacı çiz
    for node in rrt_star.node_list:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-c", alpha=0.3)

    # Optimal Yolu Çizdir
    if path is None:
        print("Hedefe ulaşılamadı!")
    else:
        path_x = [x for (x, y) in path]
        path_y = [y for (x, y) in path]
        # Mavi ve kalın çizgi ile Optimal Yolu göster
        plt.plot(path_x, path_y, '-b', linewidth=3, label="Optimal RRT* Path")
        plt.legend()

    plt.title("RRT* Optimal Path Planning")
    plt.pause(0.001)
    plt.show()

if __name__ == '__main__':
    main()
