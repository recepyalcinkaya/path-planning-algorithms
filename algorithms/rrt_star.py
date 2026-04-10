import math
import random

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None
        self.cost = 0.0  # RRT*'ın en büyük farkı: Başlangıçtan buraya olan toplam mesafe (maliyet)

class RRTStarPlanner:
    def __init__(self, obstacle_x, obstacle_y, rand_area, expand_dis=3.0, 
                 path_resolution=0.5, goal_sample_rate=10, max_iter=300, 
                 connect_circle_dist=15.0, robot_radius=1.0):
        self.obstacle_x = obstacle_x
        self.obstacle_y = obstacle_y
        self.min_rand, self.max_rand = rand_area
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.connect_circle_dist = connect_circle_dist  # Yeniden kablolama için arama yarıçapı
        self.robot_radius = robot_radius
        self.node_list = []

    def planning(self, start_x, start_y, goal_x, goal_y, animation=True):
        self.start = Node(start_x, start_y)
        self.end = Node(goal_x, goal_y)
        self.node_list = [self.start]

        for i in range(self.max_iter):
            # 1. Rastgele nokta üret
            rnd = self.get_random_node()
            
            # 2. En yakın düğümü bul
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            nearest_node = self.node_list[nearest_ind]

            # 3. Yeni düğümü oluştur
            new_node = self.steer(nearest_node, rnd, self.expand_dis)

            # 4. Çarpışma kontrolü
            if self.check_collision(new_node):
                # RRT* FARKI 1: En iyi ebeveyni bul (Choose Parent)
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)
                
                if new_node:
                    self.node_list.append(new_node)
                    # RRT* FARKI 2: Ağacı yeniden kablola (Rewire)
                    self.rewire(new_node, near_inds)

        # Optimizasyon bittikten sonra hedefe en yakın ve en düşük maliyetli rotayı bul
        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        print("Yol bulunamadı!")
        return None

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return new_node

        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))

        min_cost = min(costs)
        if min_cost == float("inf"):
            print("Uygun ebeveyn yok!")
            return new_node

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost
        return new_node

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            
            edge_node.cost = self.calc_new_cost(new_node, near_node)
            
            # Eğer yeni oluşan yol, eskisinden daha ucuzsa (kısaysa), ebeveyni değiştir!
            if self.check_collision(edge_node) and near_node.cost > edge_node.cost:
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(self.node_list[i])

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_list if i <= self.expand_dis]

        safe_goal_inds = []
        for i in goal_inds:
            t_node = self.steer(self.node_list[i], self.end)
            if self.check_collision(t_node):
                safe_goal_inds.append(i)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i
        return None

    def propagate_cost_to_leaves(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        new_node.path_x, new_node.path_y = [new_node.x], [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        new_node.parent = from_node
        return new_node

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = Node(random.uniform(self.min_rand, self.max_rand), random.uniform(self.min_rand, self.max_rand))
        else:
            rnd = Node(self.end.x, self.end.y)
        return rnd

    def check_collision(self, node):
        if node is None:
            return False
        for (ox, oy) in zip(self.obstacle_x, self.obstacle_y):
            dx_list, dy_list = [ox - x for x in node.path_x], [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]
            if min(d_list) <= self.robot_radius ** 2:
                return False 
        return True

    def calc_dist_to_goal(self, x, y):
        return math.hypot(x - self.end.x, y - self.end.y)

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in node_list]
        return dlist.index(min(dlist))

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx, dy = to_node.x - from_node.x, to_node.y - from_node.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
