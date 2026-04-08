import math
import heapq

class Node:
    def __init__(self, x, y, yaw, cost, parent_index, steer=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw      # Aracın yönelimi (radyan)
        self.cost = cost
        self.parent_index = parent_index
        self.steer = steer  # Bu düğüme gelmek için uygulanan direksiyon açısı

class HybridAStarPlanner:
    def __init__(self, obstacle_x, obstacle_y, grid_resolution=2.0, yaw_resolution=15.0):
        self.resolution = grid_resolution
        self.yaw_resolution = yaw_resolution * math.pi / 180.0
        
        # Araç Kinematik Parametreleri (Bicycle Model)
        self.WB = 2.5       # Dingil mesafesi (Wheelbase) [m]
        self.MAX_STEER = 40.0 * math.pi / 180.0 # Maksimum direksiyon açısı [rad]
        self.STEP_SIZE = 1.5 # Her dalın simülasyon adımı [m]
        self.STEER_STEPS = 3 # Kaç farklı direksiyon açısı denenecek (Sol, Düz, Sağ)
        self.robot_radius = 1.5 
        
        self.calc_obstacle_map(obstacle_x, obstacle_y)

    def planning(self, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw):
        start_node = Node(start_x, start_y, start_yaw, 0.0, -1)
        goal_node = Node(goal_x, goal_y, goal_yaw, 0.0, -1)

        open_set, closed_set = dict(), dict()
        start_id = self.calc_3d_index(start_node)
        open_set[start_id] = start_node

        pq = []
        heapq.heappush(pq, (self.calc_heuristic(start_node, goal_node), start_id))

        print("Hybrid A* Planlaması Başladı...")

        while True:
            if not open_set:
                print("Yol bulunamadı!")
                return [], [], []

            _, current_id = heapq.heappop(pq)
            
            if current_id in closed_set:
                continue

            current = open_set[current_id]

            # Hedefe yeterince yaklaşıldı mı?
            dist_to_goal = math.hypot(current.x - goal_node.x, current.y - goal_node.y)
            if dist_to_goal <= self.STEP_SIZE:
                print("Hedef Bulundu!")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                goal_node.x, goal_node.y, goal_node.yaw = current.x, current.y, current.yaw
                break

            del open_set[current_id]
            closed_set[current_id] = current

            # Kinematik modele göre dallanma (Sol, Düz, Sağ)
            for steer in self.get_steer_inputs():
                node = self.calc_next_node(current, current_id, steer)

                if not self.verify_node(node):
                    continue

                n_id = self.calc_3d_index(node)

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                    heapq.heappush(pq, (node.cost + self.calc_heuristic(node, goal_node), n_id))
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node
                        heapq.heappush(pq, (node.cost + self.calc_heuristic(node, goal_node), n_id))

        rx, ry, ryaw = self.calc_final_path(goal_node, closed_set)
        return rx, ry, ryaw

    def calc_next_node(self, current, parent_index, steer):
        # Kinematik Bisiklet Modeli (Bicycle Model) ile yeni konumu hesapla
        x = current.x + self.STEP_SIZE * math.cos(current.yaw)
        y = current.y + self.STEP_SIZE * math.sin(current.yaw)
        yaw = current.yaw + (self.STEP_SIZE / self.WB) * math.tan(steer)
        
        # Yönelimi -pi ile pi arasında tut
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        
        # Maliyeti hesapla (Dönüşleri cezalandır ki düz gitmeyi tercih etsin)
        cost = current.cost + self.STEP_SIZE + abs(steer) * 0.5
        
        return Node(x, y, yaw, cost, parent_index, steer)

    def get_steer_inputs(self):
        # Direksiyon seçenekleri: Örn; -40 derece, 0 derece, +40 derece
        steer_inputs = []
        for i in range(self.STEER_STEPS):
            steer = -self.MAX_STEER + i * (2.0 * self.MAX_STEER / (self.STEER_STEPS - 1))
            steer_inputs.append(steer)
        return steer_inputs

    def calc_final_path(self, goal_node, closed_set):
        rx, ry, ryaw = [goal_node.x], [goal_node.y], [goal_node.yaw]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            ryaw.append(n.yaw)
            parent_index = n.parent_index
        rx.reverse()
        ry.reverse()
        ryaw.reverse()
        return rx, ry, ryaw

    def calc_heuristic(self, n1, n2):
        # En kısa mesafe heuristiği
        return math.hypot(n1.x - n2.x, n1.y - n2.y)

    def calc_3d_index(self, node):
        x_idx = round((node.x - self.min_x) / self.resolution)
        y_idx = round((node.y - self.min_y) / self.resolution)
        yaw_idx = round(node.yaw / self.yaw_resolution)
        return f"{x_idx}_{y_idx}_{yaw_idx}"

    def verify_node(self, node):
        if node.x < self.min_x or node.y < self.min_y or node.x >= self.max_x or node.y >= self.max_y:
            return False

        x_idx = round((node.x - self.min_x) / self.resolution)
        y_idx = round((node.y - self.min_y) / self.resolution)

        if 0 <= x_idx < self.x_width and 0 <= y_idx < self.y_width:
            if self.obstacle_map[x_idx][y_idx]:
                return False
        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox)) - 5
        self.min_y = round(min(oy)) - 5
        self.max_x = round(max(ox)) + 5
        self.max_y = round(max(oy)) + 5

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = ix * self.resolution + self.min_x
            for iy in range(self.y_width):
                y = iy * self.resolution + self.min_y
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break
