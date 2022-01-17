from collision import is_invertible, isCollisionFree
import numpy as np

def euclidean(x1, y1, x2, y2):
    return ((x2-x1)** 2 + (y2-y1)**2) ** (1/2)

class Node:
    def __init__(self, point, parent):
        self.point = point
        self.parent = parent
        self.children = []

class Tree:
    def __init__(self, robot, obstacles, start, goal):
        self.tree_nodes = {}
        self.cost_tree_nodes = {}
        self.root = Node(start, None)
        self.tree_nodes[start] = self.root
        self.cost_tree_nodes[start] = 0        
        self.goal = Node(goal, None)
        self.robot = robot
        self.obstacles = obstacles

    def get_cost(self, point):
        return self.cost_tree_nodes[tuple(point)]
    
    def add(self, point1, point2):
        parent_node = self.tree_nodes[point1]
        if not self.exists(point2):
            new_node = Node(point2, parent_node)
            parent_node.children.append(point2)
            self.tree_nodes[point2] = new_node
            self.cost_tree_nodes[point2] = self.cost_tree_nodes[point1] + euclidean(*point1, *point2)
        else:
            new_node = self.tree_nodes[point2]
            new_node.parent.children.remove(point2)
            new_node.parent = parent_node
            parent_node.children.append(point2)
            self.cost_tree_nodes[point2] = self.cost_tree_nodes[point1] + euclidean(*point1, *point2)

    def exists(self, point):
        if point in self.tree_nodes.keys():
            return True
        return False
    
    def parent(self, point):
        if self.tree_nodes[point].parent:
            return self.tree_nodes[point].parent.point
        else:
            return None

    def get_minimum_cost_point(self, l_points):
        value = np.inf
        cheapest_point = None
        for pt in l_points:
            if self.cost_tree_nodes[pt] < value:
                value = self.cost_tree_nodes[pt]
                cheapest_point = pt
        return cheapest_point, value

    def get_nearest(self, point, l_points, r=np.inf):
        points_inside_r = []
        min_dist = 9999999
        min_point = None
        for k in l_points:
            d = euclidean(*k, *point)
            if d <= r :
                points_inside_r.append(k)
                if d < min_dist:
                    min_dist = d
                    min_point = k
        return points_inside_r, min_point
    
    def nearest(self, point, r=np.inf):
        return self.get_nearest(point, self.tree_nodes.keys(), r)
    
    def get_intersection_points(self, point1, point2):
        intersection = []
        p1, p2 = point1, point2
        x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
        edge = [y1-y2, x2-x1, x2*y1 - x1*y2]

        for obs in [*self.obstacles, [(0, 0), (10, 0)], [(10, 0), (10, 10)], [(10, 10), (0, 10)], [(0, 10), (0, 0)]]:
            for i in range(len(obs)):
                p1, p2 = obs[i], obs[(i+1)% len(obs)]
                x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
                A = [edge[:-1], [y1-y2, x2-x1]]
                b = [edge[-1], x2*y1 - x1*y2]
                if not is_invertible(A):
                    continue
                inter = np.around(np.matmul(np.linalg.inv(A),  np.array(b)), 5)
                ub_x, lb_x = max(x1, x2), min(x1, x2)
                ub_y, lb_y = max(y1, y2), min(y1, y2)
                if inter[0] >= lb_x and inter[0] <= ub_x and inter[1] >= lb_y and inter[1] <= ub_y:
                    p1, p2 = point1, point2
                    x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
                    ub_x, lb_x = max(x1, x2), min(x1, x2)
                    ub_y, lb_y = max(y1, y2), min(y1, y2)
                    if inter[0] >= lb_x and inter[0] <= ub_x and inter[1] >= lb_y and inter[1] <= ub_y:
                        if tuple(inter) != point1:
                            intersection.append(inter)
        return intersection

    def check_for_collision(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        
        mid_x, mid_y = (x1+x2)/2, (y1+y2)/2
        # Check if x is present at mid
        if not isCollisionFree(self.robot, (mid_x, mid_y), self.obstacles):
            return False
        return True
    
    
    def extend(self, point1, point2):
        # print(point1, point2)
        intersections = self.get_intersection_points(point1, point2)
        collision_free = isCollisionFree(self.robot, point2, self.obstacles)
        # print(intersections, collision_free)
        if len(intersections) == 0 and collision_free:
            self.add(tuple(point1), tuple(point2))
            return point2
        elif len(intersections) != 0 and collision_free:
            _, new_point2 = self.get_nearest(point1, intersections)
            # print(point1 in self.tree_nodes.keys(), self.tree_nodes[point1].parent)
            if (point1 in self.tree_nodes.keys()) and self.tree_nodes[point1].parent:
                # print(tuple(new_point2), tuple(self.tree_nodes[point1].parent.point))
                while tuple(new_point2) == tuple(self.tree_nodes[point1].parent.point):
                    print('here')
                    intersections.remove(new_point2)
                    _, new_point2 = self.get_nearest(point1, intersections)
                
            if self.check_for_collision(point1, new_point2):
                self.add(tuple(point1), tuple(new_point2))
                return new_point2
            else:
                return point1
        else:
            return point1

    def rewire(self, point, r):
        radius_points, _ = self.nearest(point, r)
        if len(radius_points) == 0:
            return
        
        nearest_point, cost = self.get_minimum_cost_point(radius_points)
        new_point = self.extend(nearest_point, point)
        for pt in radius_points:
            # print(tuple(new_point))
            # print(self.cost_tree_nodes[tuple(new_point)] + euclidean(*new_point, *pt))
            if pt == nearest_point:
                continue
            if self.cost_tree_nodes[tuple(pt)] > (self.cost_tree_nodes[tuple(new_point)] + euclidean(*new_point, *pt)):
                self.extend(tuple(new_point), tuple(pt))
        return         


