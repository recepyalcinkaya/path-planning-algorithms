import math
import random

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None

class RRTPlanner:
    def __init__(self, obstacle_x, obstacle_y, rand_area, expand_dis=3.0, path_resolution=0.5, goal_sample_rate=5, max_iter=500, robot_radius=1.0):
        """
        RRT Planlayıcı Başlatma
        
        obstacle_x: Engellerin x koordinatları
        obstacle_y: Engellerin y koordinatları
        rand_area: Rastgele nokta üretilecek alanın sınırları [min, max]
        expand_dis: Her adımda ağacın uzama mesafesi [m]
        path_resolution: Çarpışma kontrolü için çözünürlük [m]
        goal_sample_rate: Doğrudan hedefe yönelme ihtimali [%]
        max_iter: Maksimum döngü sayısı
        """
        self.obstacle_x = obstacle_x
        self.obstacle_y = obstacle_y
        self.min_rand, self.max_rand = rand_area
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.robot_radius = robot_radius
        self.node_list = []

    def planning(self, start_x, start_y, goal_x, goal_y, animation=True):
        """
        RRT Path Planning
        """
        self.start = Node(start_x, start_y)
        self.end = Node(goal_x, goal_y)
        self.node_list = [self.start]

        for i in range(self.max_iter):
            # 1. Rastgele bir nokta üret (Bazen doğrudan hedefi seç)
            rnd_node = self.get_random_node()

            # 2. Ağaçtaki en yakın düğümü bul
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            # 3. O noktaya doğru yeni bir dal (node) uzat
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            # 4. Çarpışma kontrolü yap, güvenliyse ağaca ekle
            if self.check_collision(new_node):
                self.node_list.append(new_node)

            # 5. Hedefe ulaştık mı kontrol et
            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node):
                    return self.generate_final_course(len(self.node_list) - 1)

        print("Cannot find path - Yol bulunamadı (Maksimum iterasyona ulaşıldı)")
        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node
        return new_node

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = Node(random.uniform(self.min_rand, self.max_rand),
                       random.uniform(self.min_rand, self.max_rand))
        else:  # %5 ihtimalle doğrudan hedefi seç (Hızlandırmak için)
            rnd = Node(self.end.x, self.end.y)
        return rnd

    def check_collision(self, node):
        if node is None:
            return False

        for (ox, oy) in zip(self.obstacle_x, self.obstacle_y):
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= self.robot_radius ** 2:
                return False  # Çarpışma var
        return True  # Güvenli

    def generate_final_course(self, goal_ind):
        path = [
