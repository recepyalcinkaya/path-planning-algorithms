import math
import heapq

class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost  # g(n): Başlangıçtan bu düğüme kadar olan kesin maliyet
        self.parent_index = parent_index

    def __str__(self):
        return f"Node(x={self.x}, y={self.y}, cost={self.cost})"

class DijkstraPlanner:
    def __init__(self, obstacle_x, obstacle_y, resolution, robot_radius):
        """
        Dijkstra planlaması için grid (ızgara) haritayı başlatır
        """
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(obstacle_x, obstacle_y)

    def planning(self, start_x, start_y, goal_x, goal_y):
        """
        Dijkstra path planning algoritması
        """
        start_node = Node(self.calc_xy_index(start_x, self.min_x),
                          self.calc_xy_index(start_y, self.min_y), 0.0, -1)
        goal_node = Node(self.calc_xy_index(goal_x, self.min_x),
                         self.calc_xy_index(goal_y, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        # Open Set için öncelik kuyruğu (Priority Queue)
        pq = []
        # A*'dan farklı olarak burada sadece cost (maliyet) kullanıyoruz, heuristic yok
        heapq.heappush(pq, (start_node.cost, self.calc_grid_index(start_node)))

        while True:
            if not open_set:
                print("Cannot find path - Yol bulunamadı!")
                return [], []

            _, current_id = heapq.heappop(pq)
            
            if current_id in closed_set:
                continue

            current = open_set[current_id]

            # Hedefe ulaştık mı kontrolü
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Goal found! - Hedef bulundu!")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Open set'ten closed set'e taşı
            del open_set[current_id]
            closed_set[current_id] = current

            # Hareket modeline göre grid'i genişlet
            for i, _ in enumerate(self.motion):
                node = Node(current.x + self.motion[i][0],
                            current.y + self.motion[i][1],
                            current.cost + self.motion[i][2], current_id)
                n_id = self.calc_grid_index(node)

                # Düğüm güvenli değilse (engel vs.) atla
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                    heapq.heappush(pq, (node.cost, n_id))
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node
                        heapq.heappush(pq, (node.cost, n_id))

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    def calc_grid_position(self, index, min_position):
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion
