from matplotlib import pyplot as plt, colors, patches
from matplotlib.path import Path
import matplotlib.animation as animation
import numpy as np
import time


FFwriter = animation.FFMpegWriter

from rrt import make_configuration_space, rrt, rrt_star

def visualize_problem(robot, obstacles, start, goal, old_obstacles = None, show=True):

    fig = plt.figure()
    ax = fig.gca()
    polygons = []

    rob = np.array(robot)

    if start:
        rob_start = rob + np.array(start)
        poly = patches.Polygon(rob_start, True, facecolor='r')
        ax.add_patch(poly)
    
    if goal:
        rob_goal = rob + np.array(goal)
        poly = patches.Polygon(rob_goal, True, facecolor='g')
        ax.add_patch(poly)

    if old_obstacles:
        for obs in old_obstacles:
            ob = np.array(obs)
            poly = patches.Polygon(ob, True, facecolor='slategrey', zorder=1)
            ax.add_patch(poly)

    for obs in obstacles:
        ob = np.array(obs)
        poly = patches.Polygon(ob, True, facecolor='pink', zorder=0, edgecolor='g')
        ax.add_patch(poly)
        # polygons.append(ob[:, 0])
        # polygons.append(ob[:, 1])
        # polygons.append('slategrey')

    # plt.fill(*polygons, zorder=0)

    ax.set_xticks(np.arange(0, 11, 1))
    ax.set_yticks(np.arange(0, 11, 1))
    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])

    if show:
        # plt.grid()
        
        plt.show()

def visualize_points(points, robot, obstacles, start, goal, old_obstacles=None, show=True, free=True):
    if type(points) == dict:
        points_arr = np.array(points['free'])
        points_collide_arr = np.array(points['collide'])
    else:
        points_arr = points
        points_collide_arr = []

    visualize_problem(robot, obstacles, start, goal, old_obstacles, show=False)
    if len(points_arr) > 0:
        plt.scatter(points_arr[:, 0], points_arr[:, 1], color='black', zorder=2)
    if len(points_collide_arr) > 0:
        plt.scatter(points_collide_arr[:, 0], points_collide_arr[:, 1], color='red', zorder=1)
    
    if show:
        # polygons = []
        # for p in [*points_arr, *points_collide_arr]:
        #     rob = np.array(robot)
        #     rob_start = rob + np.array(p)
        #     polygons.append(rob_start[:, 0])
        #     polygons.append(rob_start[:, 1])
        #     polygons.append('b')
        
        # plt.fill(*polygons, zorder=2)
        
        plt.show()

def visualize_path(robot, obstacles, path, old_obstacles=None, tree=None, show=True):
    
    if tree:
        visualize_tree(tree, robot, obstacles, path[0], path[-1], old_obstacles, show=False)
    else:
        visualize_points(np.array(path), robot, obstacles, path[0], path[-1], old_obstacles, show=False)

    for i in range(len(path) - 1):
        if path[i] and path[i+1]:
            x1, y1 = path[i]
            x2, y2 = path[i+1]
            plt.plot([x1, x2], [y1, y2], color='red')

    if show:
        
        plt.show()

def visualize_tree(tree, robot, obstacles, start, goal, old_obstacles=None, show=True):
    visualize_problem(robot, obstacles, start, goal, old_obstacles, show=False)
    points = np.array(list(tree.tree_nodes.keys()))
    if len(points) > 0:
        plt.scatter(points[:, 0], points[:, 1], color='black', zorder=1)

    closed_list = []
    open_list = [start]

    # print(open_list)
    while len(open_list) != 0:
        node = open_list.pop(0)
        # print('here', node)
        if node:
            x1, y1 = tuple(node)
            # print('heree', tree.exists((x1, y1)))
            if tree.exists((x1, y1)):
                # print('here')
                closed_list.append(node)
                for child in tree.tree_nodes[tuple(node)].children:
                    if child in closed_list:
                        continue
                    x2, y2 = child
                    # print('here')
                    plt.plot([x1, x2], [y1, y2], color='black')
                    open_list.append(child)
    if show:
        plt.show()

def visualize_configuration(robot, obstacles, start, goal):
    new_obstacles = make_configuration_space(robot, obstacles)
    visualize_problem(robot, new_obstacles, start, goal, obstacles)
    # tree, path = rrt(robot, new_obstacles, start, goal, 500, return_tree=True)

def visualize_rrt(robot, obstacles, start, goal, iter_n):
    new_obstacles = make_configuration_space(robot, obstacles)
    tree, path = rrt(robot, new_obstacles, start, goal, iter_n, return_tree=True)
    visualize_path(robot, new_obstacles, path, obstacles, tree)

def visualize_rrt_star(robot, obstacles, start, goal , iter_n):
    new_obstacles = make_configuration_space(robot, obstacles)
    tree, path = rrt_star(robot, new_obstacles, start, goal, iter_n, return_tree=True)
    visualize_path(robot, new_obstacles, path, obstacles, tree)



def animate_rrt(robot, obstacles, start, goal, iter_n, interval=50):
    new_obstacles = make_configuration_space(robot, obstacles)
    tree, path = rrt(robot, new_obstacles, start, goal, iter_n, return_tree=True)
    ani = visualize_animations(robot, new_obstacles, start, goal, path, obstacles, tree, interval)
    ani.save('1_rrt.mp4', writer = FFwriter(10))


def animate_rrt_star(robot, obstacles, start, goal, iter_n, interval=50):
    new_obstacles = make_configuration_space(robot, obstacles)
    tree, path = rrt_star(robot, new_obstacles, start, goal, iter_n, return_tree=True)
    ani = visualize_animations(robot, new_obstacles, start, goal, path, obstacles, tree, interval)
    ani.save('1_rrt_star.mp4', writer = FFwriter(20))

def visualize_animations(robot, obstacles, start, goal, path, old_obstacles, tree, interval=50):
    fig, ax = plt.subplots(nrows=1, ncols=1)
    
    # fig = plt.figure()
    # ax = fig.gca()
    polygons = []

    rob = np.array(robot)

    if start:
        rob_start = rob + np.array(start)
        poly = patches.Polygon(rob_start, True, facecolor='r')
        ax.add_patch(poly)
    
    if goal:
        rob_goal = rob + np.array(goal)
        poly = patches.Polygon(rob_goal, True, facecolor='g')
        ax.add_patch(poly)

    for obs in obstacles:
        ob = np.array(obs)
        poly = patches.Polygon(ob, True, facecolor='pink', zorder=0, edgecolor='g')
        ax.add_patch(poly)

    if old_obstacles:
        for obs in old_obstacles:
            ob = np.array(obs)
            poly = patches.Polygon(ob, True, facecolor='slategrey', zorder=1)
            ax.add_patch(poly)

    
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)

    patches_stored = {}
    switch = []

    def animate_tree(frame):
        print(frame)
        t = frame[0]
        if (type(t)==int) and (t == -1):
            switch.append(-1)
            return

        if (type(t)==int) and (t == -2):
            # switch.append(-1)
            return
        
        if len(switch) == 0:
            _, wire, point1, point2 = frame
            if wire:
                verts = [point1, point1, point2]
                codes = [Path.MOVETO, Path.LINETO, Path.LINETO]
                path = Path(verts, codes)
                patch = patches.PathPatch(path, edgecolor='black', facecolor='none', lw=1, zorder=10)
                ax.add_patch(patch)
                patches_stored[(point1, point2)] = patch
            
            else:
                patch = patches_stored[(point1, point2)]
                patch.remove()
        else:
            point1, point2 = list(frame[0]), list(frame[1])
            print(frame)
            verts = [point1, point1, point2]
            codes = [Path.MOVETO, Path.LINETO, Path.LINETO]
            path = Path(verts, codes)
            patch = patches.PathPatch(path, edgecolor='r', facecolor='none', lw=1.3, zorder=11)
            ax.add_patch(patch)

    new_path = zip(path[:-1], path[1:])
    end_count = [[-2]*1]*15
    ani_tree = animation.FuncAnimation(fig, animate_tree, frames=[*tree.animation, (-1, 0, 0, 0), *new_path, *end_count], interval=interval, repeat=False, save_count=2000)

    # plt.show()

    return ani_tree
    
    
    # time.sleep(200 * len(tree.animation))

    # def animate_path(frame):
    #     point1, point2 = frame[0], frame[1]
    #     print(frame)
    #     verts = [point1, point1, point2]
    #     codes = [Path.MOVETO, Path.LINETO, Path.LINETO]
    #     path = Path(verts, codes)
    #     patch = patches.PathPatch(path, edgecolor='r', facecolor='none', lw=1, zorder=11)
    #     ax.add_patch(patch)
    
    # ani_path = animation.FuncAnimation(fig, animate_path, frames=zip(path[:-1], path[1:]), interval=100)

    # plt.show()

    

