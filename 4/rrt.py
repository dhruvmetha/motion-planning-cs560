from numpy.core.defchararray import center
from numpy.lib.arraypad import pad
from numpy.lib.shape_base import expand_dims
from collision import isCollisionFree
from sampler import sample
from tree import Tree, euclidean
from convex_hull import *
import random
import numpy as np
import math
from tqdm import tqdm


def get_bounds(points):
    ub_x = -99999
    lb_x = 99999
    ub_y = -99999
    lb_y = 99999
    for i in points:
        x, y = tuple(i)
        if x >= ub_x:
            ub_x = x
        if x <= lb_x:
            lb_x = x
        if y >= ub_y:
            ub_y = y
        if y <= lb_y:
            lb_y = y
    return lb_x, ub_x, lb_y, ub_y

def shortest_distance(x1, y1, a, b, c):
    d = abs((a * x1 + b * y1 + c)) / (math.sqrt(a * a + b * b))
    return d

def make_configuration_space(robot, obstacles):
    flipped_robot = np.concatenate((np.expand_dims(np.array(robot)[:, 0] * - 1, axis=1), np.expand_dims(np.array(robot)[:, 1], axis=1)), axis=1)

    new_obstacles = []
    for obs in obstacles:
        new_obs = []
        padded_points = []
        for i in range(len(obs)):
            p0, p1, p2 = obs[i-1], obs[i], obs[(i+1)%len(obs)]
            x0, y0, x1, y1, x2, y2 = p0[0], p0[1], p1[0], p1[1], p2[0], p2[1]
            a1, b1, c1 = y0-y1, x1-x0, -x1*y0 + x0*y1
            a2, b2, c2 = y1-y2, x2-x1, - x2*y1 + x1*y2
            flipped_robot_p1 = np.array(flipped_robot) + np.array(p1)
            potential_obs = []
            distances = []
            for pos in flipped_robot_p1:
                x, y = pos[0], pos[1]
                d = shortest_distance(x, y, a2, b2, c2)
                distances.append(d)
            distances = np.array(distances)
            potential_obs.extend(flipped_robot_p1[np.argwhere(np.array(distances) == np.max(distances)).flatten()].tolist())
            distances = []
            for pos in flipped_robot_p1:
                x, y = pos[0], pos[1]
                d = shortest_distance(x, y, a1, b1, c1)
                distances.append(d)
            distances = np.array(distances)
            potential_obs.extend(flipped_robot_p1[np.argwhere(np.array(distances) == np.max(distances)).flatten()].tolist())
            
            
            for pot_obs in potential_obs:
                if pot_obs in padded_points or not isCollisionFree(flipped_robot, pot_obs, obstacles):
                    continue  
                padded_points.append(pot_obs)
            new_obs.extend(padded_points)
            new_obs.extend(obs)

        keep_obs = []
        for i in new_obs:
            if i not in keep_obs:
                keep_obs.append(i)

        # print(new_obs)

        # lb_x, ub_x, lb_y, ub_y = get_bounds(new_obs)

        # keep_obs = []
        # for i in new_obs:
        #     if i in padded_points:
        #         if i not in keep_obs:
        #             keep_obs.append(i)
        #     if i in obs:
        #         x, y = i[0], i[1]
        #         if x == lb_x or x == ub_x or y == lb_y or y == ub_y or True:
        #             keep_obs.append(i)

        keep_obs = convexHull(keep_obs)

        keep_obs = np.array(keep_obs)
        centroid = np.mean(keep_obs, axis=0).tolist()

        keep_obs = keep_obs.tolist()
        keep_obs.append(centroid)
        keep_obs = np.array(keep_obs) - np.array(centroid)

        keep_obs = sorted(keep_obs.tolist(), key=lambda p: math.atan2(p[1], p[0]))
        keep_obs.remove([0, 0])
        keep_obs = np.array(keep_obs) + np.array(centroid)
        new_obstacles.append(keep_obs.tolist())

    return new_obstacles

def rrt(robot, obstacles, start, goal, iter_n, return_tree=False):
    tree = Tree(robot, obstacles, tuple(start), tuple(goal))
    last_point = tuple(start)
    n1, n2 = 500, 2000
    dt = 1e-4
    # nearest = set()
    # nearest.add(last_point)
    for _ in tqdm(range(iter_n)):
        next_point = tuple(goal)
        if random.random() < 0.8:
            next_point = tuple(sample())

        # print(next_point)
        # _, nearest = tree.get_nearest(next_point, list(tree.tree_nodes.keys()))
        # _, last_point = tree.get_nearest(tuple(goal), list(nearest))
        # print(last_point)
        extended_point = tree.extend(next_point, n1, n2, dt)
        if tuple(extended_point) == tuple(next_point):
            continue
        # if tuple(extended_point) == tuple(goal):
        #     break
        # print(len(trajectory))
        # if len(trajectory) > 0:
        # print()
        # print(euclidean(*extended_point[:-1], 0, *goal[:-1], 0))
        if euclidean(*extended_point[:-1], 0, *goal[:-1], 0) < 1:
            goal = tuple(extended_point)
            break
            # last_point = tuple()
            # nearest.add(tuple(trajectory[-1]))
            # print(len(nearest))

        # if last_point == tuple(goal):
            # break

    path = []
    p = goal
    k = 0
    while True:
        if not tree.exists(tuple(p)):
            path.extend([None, start])
            break

        # if k > 10:
        #     break
        # k += 1

        path.append(p)
        parent = tree.parent(tuple(p))
        print(p, parent)
        if not parent:
            break

        # controls = [tree.controls[tuple(parent)]]
        # duration = [tree.durations[tuple(parent)]]
        # discretized = 20
        # temp_p = []

        # c, d = [*controls], [*duration]

        # q_near = parent
        # _, q_near_dot = robot.propagate(q_near, controls, duration, dt)
        # for _ in range(discretized):
        #     controls, duration = [*c], [*d]
        #     q_near, _ = robot.propagate(q_near, controls, duration, dt/discretized, q_near_dot)
        #     # print(q_near)
        #     temp_p.append(tuple(q_near.tolist()))
        # print(temp_p)
        
        # path.extend(temp_p[::-1])

        p = parent

    # path = [None, start]

    print(path)

    if return_tree:
        return tree, path[::-1]
    else:
        return None, path[::-1]    

def rrt_star(robot, obstacles, start, goal, iter_n, return_tree=False):
    tree = Tree(robot, obstacles, tuple(start), tuple(goal))
    # last_point = tuple(start)
    for _ in tqdm(range(iter_n)):
        next_point = goal
        if random.random() < 0.6:
            next_point = tuple(sample())

        tree.rewire(next_point, 3)

    path = []
    p = goal
    while True:
        if not tree.exists(tuple(p)):
            path.extend([None, start])
            break
        path.append(p)
        p = tree.parent(tuple(p))
        if not p:
            break 

    if return_tree:
        return tree, path[::-1]
    else:
        return None, path[::-1]  
