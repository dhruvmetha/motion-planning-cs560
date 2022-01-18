from numpy.core.numeric import ones
from sampler import sample_between, sample_integer
from collision import is_invertible, isCollisionFree
import numpy as np
import math
from visualizer import *

def euclidean(x1, y1, a1, x2, y2, a2):

    if a1 == 0 or a2 == 0:
        pass
    else:
        a1 = math.atan2(np.sin(a1), np.cos(a1))
        a2 = math.atan2(np.sin(a2), np.cos(a2))

    if a1 < 0:
        a1 = 2 * np.pi + a1
    if a2  < 0:
        a2 = 2*np.pi + a2
    ang1 = a2 - a1
    ang2 = (2 * np.pi) - (a2 - a1)
    if abs(ang1) > abs(ang2):
        d = ang2
    else:
        d = ang1
    
    return ((x2-x1)** 2 + (y2-y1)**2 + d**2) ** (1/2)

class Node:
    def __init__(self, point, parent):
        self.point = point
        self.parent = parent
        self.children = []

class Tree:
    def __init__(self, robot, obstacles, start, goal):
        self.tree_nodes = {}
        self.controls = {}
        self.durations = {}
        self.cost_tree_nodes = {}
        self.root = Node(start, None)
        self.tree_nodes[start] = self.root
        self.cost_tree_nodes[start] = 0        
        self.goal = Node(goal, None)
        self.robot = robot
        self.obstacles = obstacles
        self.animation = []
        self.timestamp = 0

    def get_cost(self, point):
        return self.cost_tree_nodes[tuple(point)]
    
    def add(self, point1, point2, control=None, duration=None):
        parent_node = self.tree_nodes[point1]
        if not self.exists(point2):
            new_node = Node(point2, parent_node)
            parent_node.children.append(point2)
            self.tree_nodes[point2] = new_node
            self.cost_tree_nodes[point2] = self.cost_tree_nodes[point1] + euclidean(*point1, *point2)
            if control is not None:
                self.controls[point1] = control
            if duration is not None:
                self.durations[point1] = duration
            self.animation.append((self.timestamp, True, point1, point2))
                
        else:
            new_node = self.tree_nodes[point2]
            if self.exists(new_node.parent):
                new_node.parent.children.remove(point2)
                new_node.parent = parent_node
                parent_node.children.append(point2)
                self.cost_tree_nodes[point2] = self.cost_tree_nodes[point1] + euclidean(*point1, *point2)
                if control is not None:
                    self.controls[point1] = control
                if duration is not None:
                    self.durations[point1] = duration

    def exists(self, point):

        try:
            if self.tree_nodes[point]:
                return True
        except KeyError:
            return False

        return False


        # if point in self.tree_nodes.keys():
        #     return True
        # return False
    
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
        points_inside_r_unsorted = []
        min_dist = 9999999
        min_point = None
        for k in l_points:
            d = euclidean(*k, *point)
            if d <= r :
                points_inside_r_unsorted.append((k, d))
                if d < min_dist:
                    min_dist = d
                    min_point = k
        points_inside_r = [i[0] for i in list(sorted(points_inside_r_unsorted, key= lambda x: x[1]))]
        # print([i for i in list(sorted(points_inside_r_unsorted, key= lambda x: x[1]))])
        # print()
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
                            intersection.append([*inter, 0.0])
        return intersection

    def check_for_collision(self, p1, p2):
        x1, y1, a1 = p1
        x2, y2, a2 = p2

        number_of_points=10
        xs=np.linspace(x1,x2,number_of_points+2)
        ys=np.linspace(y1,y2,number_of_points+2)

        last_know_point = (x1, y1, a1)

        #print them
        for i, j in zip(xs, ys):
            if not isCollisionFree(self.robot, (i, j, a2), self.obstacles):
                return last_know_point
            last_know_point = (i, j, a2)
        return last_know_point
    
    def extend(self, point, n1, n2, dt):
        u_low, u_high = -5, 5
        N = 5
        q = point

        if not isCollisionFree(self.robot, q, self.obstacles):
            return point

        # print('new sample', q)
        # print('tree length', len(self.tree_nodes.keys()))
        points_in_r, nearest_node = self.nearest(q)
        # print('nearest_node', nearest_node, points_in_r)
        # print('nearest', nearest_node)
        # controls = []
        # durations = []
        nearest_nodes = {}
        j = 0
        k = 0
        for _ in range(5):
            # controls.append(sample_between(u_low, u_high, size=2))
            # durations.append(sample_integer(n1, n2))
            controls = [sample_between(u_low, u_high, size=2)]
            duration = [sample_integer(n1, n2)]
            c, d = [*controls], [*duration]
            q_new, q_new_dot = self.robot.propagate(nearest_node, controls, duration, dt)
            q_new = tuple(q_new)
            q_new_dot = q_new_dot
            collider = False
            # print('collision', isCollisionFree(self.robot, q_new, self.obstacles))
            if isCollisionFree(self.robot, q_new, self.obstacles):
                # q_near = nearest_node
                # discretized = 20
                # for i in range(discretized):
                #     controls, duration = [*c], [*d]
                #     q_near, q_new_dot = self.robot.propagate(q_near, controls, duration, dt/discretized, q_new_dot)
                #     if isCollisionFree(self.robot, q_near, self.obstacles):
                #         continue
                #     else:
                #         # visualize_problem(self.robot, self.obstacles, tuple(q_near), (0, 0, 0))
                #         # print('collision', q_near)
                #         collider = True
                #         break
                if not collider:
                    # print('adding', q_new)
                    nearest_nodes[q_new] = (c[0], d[0])
                    j += 1
                # else:
                #     q_new = q_near
                #     print('adding', q_new, c, d)
                #     nearest_nodes[q_new] = (c[0], d[0])
                #     j += 1
        if len(nearest_nodes.keys()) > 0:
            _, new_node = self.get_nearest(q, nearest_nodes.keys())
            if new_node:
                # print(nearest_node)
                # print('adding', new_node, isCollisionFree(self.robot, new_node, self.obstacles))
                self.add(nearest_node, new_node, *nearest_nodes[new_node])
                return new_node
        return point
    # def extend(self, point1, point2):
    #     # print(point1, point2)
    #     intersections = self.get_intersection_points(point1, point2)
    #     collision_free = isCollisionFree(self.robot, point2, self.obstacles)
    #     # print(intersections, collision_free)
    #     if len(intersections) == 0 and collision_free:
    #         new_point_inter = self.check_for_collision(point1, point2)
    #         # if self.check_for_collision(point1, point2):
    #         self.add(tuple(point1), tuple(new_point_inter))
    #         return new_point_inter
    #         # else:
    #             # return point1
    #     elif len(intersections) != 0 and collision_free:
    #         l_points, new_point2 = self.get_nearest(point1, intersections)
    #         # print(l_points, new_point2)
    #         # print(point1 in self.tree_nodes.keys(), self.tree_nodes[point1].parent)
    #         if (point1 in self.tree_nodes.keys()) and self.tree_nodes[point1].parent:
    #             # print(tuple(new_point2), tuple(self.tree_nodes[point1].parent.point))
    #             while tuple(new_point2) == tuple(self.tree_nodes[point1].parent.point):
    #                 print('here')
    #                 intersections.remove(new_point2)
    #                 l_points.remove(new_point2)

    #                 new_point2 = l_points
    #                 # _, new_point2 = self.get_nearest(point1, intersections)
                
            
    #         new_point_inter = self.check_for_collision(point1, new_point2)
    #         self.add(tuple(point1), tuple(new_point_inter))
    #         return new_point_inter
    #     else:
    #         return point1

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


